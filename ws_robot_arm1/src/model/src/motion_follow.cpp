#include "model/motion_follow.h"

// 主电机当前位置接收回调
// 功能：将主机械臂电机当前位置作为从机械臂电机目标位置发布出去
void motion_simulation::MasterPresentPositionsRead(const sensor_msgs::JointState::ConstPtr &jointstate)
{
    trajectory_msgs::JointTrajectory JointTrajectory;
    JointTrajectory.header.stamp = ros::Time::now()+ros::Duration(0.003);

    JointTrajectory.joint_names.resize(dxl_num*2);
    JointTrajectory.points.resize(1);
    JointTrajectory.points[0].positions.resize(dxl_num*2);
    for(size_t i = 0; i < dxl_num*2; i++)
    {
        if(i<dxl_num)
        {
            std::string str = std::to_string(i+1);
            JointTrajectory.joint_names[i] = "link3_" + str + "_joint";
        }
        else
        {
            std::string str = std::to_string(i+1 - dxl_num);
            JointTrajectory.joint_names[i] = "link4_" + str + "_joint";
        }
        JointTrajectory.points[0].positions[i] = jointstate->position[i];
    }
    JointTrajectory.points[0].time_from_start = ros::Duration(0.04);

    slave_goal_positions_pub.publish(JointTrajectory);
}

// 休眠n毫秒
void delay_ms(int milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

// 有参构造
motion_physical::motion_physical(ros::NodeHandle& nodehandle):
    nh(nodehandle),
    timer( initializeTimerDef() ),
    MasterLeftDxl( initializeArmDef("MasterLeftDxl") ),
    SlaveLeftDxl( initializeArmDef("SlaveLeftDxl") ),
    MasterRightDxl( initializeArmDef("MasterRightDxl") ),
    SlaveRightDxl( initializeArmDef("SlaveRightDxl") )
{
    if(arms_number >= 2)
    {
        config_master_dxl(MasterLeftDxl);
        config_slave_dxl(SlaveLeftDxl);
    }
    if(arms_number >= 4)
    {
        config_master_dxl(MasterRightDxl);
        config_slave_dxl(SlaveRightDxl);
    }
}

//初始化定时器
ros::Timer motion_physical::initializeTimerDef()
{
    ros::param::get("joints_number", joints_number);
    ros::param::get("arms_number", arms_number);
    ros::param::get("BAUDRATE",BAUDRATE);
    ros::param::get("control_cycle",control_cycle);

    return nh.createTimer(ros::Duration(control_cycle), &motion_physical::Timer_callback, this);
}

//初始化ArmDef类型结构体
motion_physical::ArmDef motion_physical::initializeArmDef(const std::string& paramName)
{
    ArmDef arm = {
        {1},
        "dev/ttyUSB0",
        PortHandler::getPortHandler(arm.DEVICE_NAME),
        GroupSyncWrite(arm.portHandler, packetHandler, ADDR_GOAL_POSITION, 4),
        GroupSyncRead(arm.portHandler, packetHandler, ADDR_PRESENT_POSITION, 4),
        {0}
    };

    arm.DXL_ID.resize(joints_number);
    arm.present_position.resize(arm.DXL_ID.size());

    string s = "";
    ros::param::get(paramName + "/DEVICE_NAME", s);
    arm.DEVICE_NAME = s.c_str();
    ros::param::get(paramName + "/DXL_ID", arm.DXL_ID);

    arm.portHandler = PortHandler::getPortHandler(arm.DEVICE_NAME);
    arm.groupSyncWrite = GroupSyncWrite(arm.portHandler, packetHandler, ADDR_GOAL_POSITION, 4);
    arm.groupSyncRead = GroupSyncRead(arm.portHandler, packetHandler, ADDR_PRESENT_POSITION, 4);

    return arm;
}

// 发送目标位置
// 功能：将组写电机位置功能打包
void motion_physical::dxl_positions_tx(ArmDef& Arm, const vector<int32_t> &slave_position)
{
    uint8_t param_goal_position[4];
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    for(size_t i=0; i<Arm.DXL_ID.size(); i++)
    {
        param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(slave_position[i]));
        param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(slave_position[i]));
        param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(slave_position[i]));
        param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(slave_position[i]));

        dxl_addparam_result = Arm.groupSyncWrite.addParam(Arm.DXL_ID[i], param_goal_position);
        if (dxl_addparam_result != true) {
            ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", Arm.DXL_ID[i]);
        }
    }

    dxl_comm_result = Arm.groupSyncWrite.txPacket();
    if (dxl_comm_result == COMM_SUCCESS){
        ROS_INFO("setPosition success");
    } else {
        ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
    }

    Arm.groupSyncWrite.clearParam();
}

// 接收当前位置
// 功能：将组写当前位置功能打包
void motion_physical::dxl_positions_txRx(ArmDef& Arm)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    int32_t position = 0;

    dxl_comm_result = Arm.groupSyncRead.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("Failed to get position! Result: %d", dxl_comm_result);
        Arm.groupSyncRead.clearParam();
    }
    else
    {
        Arm.present_position.clear();
        for(size_t i=0;i<Arm.DXL_ID.size();i++)
        {
            position = Arm.groupSyncRead.getData(Arm.DXL_ID[i], ADDR_PRESENT_POSITION, 4);
            Arm.present_position.push_back(position);
            ROS_INFO("[ID:%03d] getPosition : [POSITION:%d]", Arm.DXL_ID[i], Arm.present_position[i]);
        }
    }
}

// 机械臂回到初始位置
// 功能：将机械臂回零功能打包
void motion_physical::homing(ArmDef& Arm)
{
    vector<int32_t> zero = {0,0,0,0,0,0};
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;
    uint8_t dxl_model_number = 0;

    if (!Arm.portHandler->openPort())
    {
        ROS_ERROR("Failed to open the port!");
    }
    if (!Arm.portHandler->setBaudRate(BAUDRATE))
    {
        ROS_ERROR("Failed to set the baudrate!");
    }

    for(size_t i=0; i<Arm.DXL_ID.size(); i++)
    {
        // ping
        packetHandler->ping(Arm.portHandler, Arm.DXL_ID[i], (uint16_t*)&dxl_model_number, &dxl_error);
        if(dxl_error != 0)
            printf("%s", packetHandler->getRxPacketError(dxl_error));
        else
            printf("[ID:%03d] ping Succeeded. Dynamixel model number : %d",Arm.DXL_ID[i], dxl_model_number);

        // disable Dynamixel Torque
        packetHandler->write1ByteTxRx(Arm.portHandler, Arm.DXL_ID[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if(dxl_error != 0)
            printf("%s", packetHandler->getRxPacketError(dxl_error));
        else
            printf("[ID:%03d] disable Dynamixel Torque success", Arm.DXL_ID[i]);

        // Enable Dynamixel Torque
        packetHandler->write1ByteTxRx(Arm.portHandler, Arm.DXL_ID[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if(dxl_error != 0)
            printf("%s", packetHandler->getRxPacketError(dxl_error));
        else
            printf("[ID:%03d] Enable Dynamixel Torque success", Arm.DXL_ID[i]);

        // add groupSyncRead storage
        dxl_addparam_result = Arm.groupSyncRead.addParam(Arm.DXL_ID[i]);
        if (dxl_addparam_result != true)
        {
            ROS_ERROR("Failed to addparam to groupSyncRead for Dynamixel ID %d", Arm.DXL_ID[i]);
        }
    }
    
    delay_ms(50);

    dxl_positions_tx(Arm, zero);
    delay_ms(10);
    while(1)
    {
        dxl_positions_txRx(Arm);

        uint8_t num = 0;
        for(size_t i=0;i<Arm.DXL_ID.size();i++)
        {
            if(abs(Arm.present_position[i]-zero[i]) < DXL_MOVING_STATUS_THRESHOLD)
            {
                num++;
            }
        }
        if(num == Arm.DXL_ID.size())
        {
            break;
        }
        delay_ms(10);
    }
}

// 配置从机械臂的电机并使其回零
void motion_physical::config_slave_dxl(ArmDef& Arm)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    // Return to zero
    homing(Arm);

    for(size_t i=0; i<Arm.DXL_ID.size(); i++)
    {
        // disable Dynamixel Torque
        packetHandler->write1ByteTxRx(Arm.portHandler, Arm.DXL_ID[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if(dxl_error != 0)
            printf("%s", packetHandler->getRxPacketError(dxl_error));
        else
            printf("[ID:%03d] disable Dynamixel Torque success", Arm.DXL_ID[i]);
        delay_ms(50);

        // Disable trajectory profile
        packetHandler->write1ByteTxRx(Arm.portHandler, Arm.DXL_ID[i], ADDR_DRIVE_MODE, PROFILE_DISABLE, &dxl_error);
        if(dxl_error != 0)
            printf("%s", packetHandler->getRxPacketError(dxl_error));
        else
            printf("[ID:%03d] Disable trajectory profile", Arm.DXL_ID[i]);
        delay_ms(50);

        // Enable Dynamixel Torque
        packetHandler->write1ByteTxRx(Arm.portHandler, Arm.DXL_ID[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if(dxl_error != 0)
            printf("%s", packetHandler->getRxPacketError(dxl_error));
        else
            printf("[ID:%03d] Enable Dynamixel Torque success", Arm.DXL_ID[i]);
        delay_ms(50);
    }
}

// 配置主机械臂的电机并使其回零
void motion_physical::config_master_dxl(ArmDef& Arm)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    // Return to zero
    homing(Arm);

    for(size_t i=0; i<Arm.DXL_ID.size(); i++)
    {
        // disable Dynamixel Torque
        packetHandler->write1ByteTxRx(Arm.portHandler, Arm.DXL_ID[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if(dxl_error != 0)
            printf("%s", packetHandler->getRxPacketError(dxl_error));
        else
            printf("[ID:%03d] disable Dynamixel Torque success", Arm.DXL_ID[i]);

        // set operator mode to current
        packetHandler->write1ByteTxRx(Arm.portHandler, Arm.DXL_ID[i], ADDR_OPERATOR_MODE, CURRENT_MODE, &dxl_error);
        if(dxl_error != 0)
            printf("%s", packetHandler->getRxPacketError(dxl_error));
        else
            printf("[ID:%03d] set operator mode to current success", Arm.DXL_ID[i]);

        // Enable Dynamixel Torque
        packetHandler->write1ByteTxRx(Arm.portHandler, Arm.DXL_ID[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if(dxl_error != 0)
            printf("%s", packetHandler->getRxPacketError(dxl_error));
        else
            printf("[ID:%03d] Enable Dynamixel Torque success", Arm.DXL_ID[i]);

        // set goal cerrnt to 0
        packetHandler->write4ByteTxRx(Arm.portHandler, Arm.DXL_ID[i], ADDR_GOAL_CURRENT, 0, &dxl_error);
        if(dxl_error != 0)
            printf("%s", packetHandler->getRxPacketError(dxl_error));
        else
            printf("[ID:%03d] set goal cerrnt to 0 success", Arm.DXL_ID[i]);
    }
}

// 关节状态发布
void motion_physical::joints_state_publish(ArmDef& Arm, string robot_ref)
{
    sensor_msgs::JointState JointState;

    JointState.header.stamp = ros::Time::now();
    if(joints_number >= 1)
        JointState.name.push_back("link" + robot_ref + "_1_joint");
    if(joints_number >= 2)
        JointState.name.push_back("link" + robot_ref + "_2_joint");
    if(joints_number >= 3)
        JointState.name.push_back("link" + robot_ref + "_3_joint");
    if(joints_number >= 4)
        JointState.name.push_back("link" + robot_ref + "_4_joint");
    if(joints_number >= 5)
        JointState.name.push_back("link" + robot_ref + "_5_joint");

    int num = std::stoi(robot_ref);
    num++;
    std::string str = std::to_string(num);
    if(joints_number >= 6)
        JointState.name.push_back("link" + str + "_1_joint");
    if(joints_number >= 7)
        JointState.name.push_back("link" + str + "_2_joint");
    if(joints_number >= 8)
        JointState.name.push_back("link" + str + "_3_joint");
    if(joints_number >= 9)
        JointState.name.push_back("link" + str + "_4_joint");
    if(joints_number >= 10)
        JointState.name.push_back("link" + str + "_5_joint");
    
    for(size_t i=0; i<Arm.DXL_ID.size(); i++)
    {
        JointState.position.push_back(Arm.present_position[i]/pow(2,15) * M_PI * 2);
    }
    joints_state_pub.publish(JointState);
}

// 将电机当前位置读取到present_position容器中
void motion_physical::PositionsRead()
{
    if(arms_number >= 2)
    {
        dxl_positions_txRx(MasterLeftDxl);
        dxl_positions_txRx(SlaveLeftDxl);
        joints_state_publish(MasterLeftDxl, "1");
        joints_state_publish(SlaveLeftDxl, "3");
    }
    if(arms_number >= 4)
    {
        dxl_positions_txRx(MasterRightDxl);
        dxl_positions_txRx(SlaveRightDxl);
        joints_state_publish(MasterRightDxl, "2");
        joints_state_publish(SlaveRightDxl, "4");
    }
}

// 下发从电机当前位置到stm32中
void motion_physical::PositionsWrite()
{
    if(arms_number >= 2) dxl_positions_tx(SlaveLeftDxl, MasterLeftDxl.present_position);
    if(arms_number >= 4) dxl_positions_tx(SlaveRightDxl, MasterRightDxl.present_position);
}

// 回调函数，准时读取和发送位置
void motion_physical::Timer_callback(const ros::TimerEvent& event)
{
    PositionsRead();
    PositionsWrite();
}