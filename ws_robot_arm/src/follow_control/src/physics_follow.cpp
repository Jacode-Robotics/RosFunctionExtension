#include "../include/physics_follow.h"

// 有参构造
motion_physical::motion_physical(ros::NodeHandle& nodehandle):
    nh(nodehandle),
    timer( initializeTimerDef() ),
    MasterDxl( initializeArmDef("MasterDxl") ),
    SlaveDxl( initializeArmDef("SlaveDxl") )
{
    config_master_dxl(MasterDxl);
    config_slave_dxl(SlaveDxl);

    cbType = boost::bind(&motion_physical::dyn_cb, this, _1, _2);
    server.setCallback(cbType);     // 服务器对象调用回调对象
}

// 休眠n毫秒
void delay_ms(int milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

void motion_physical::write_Byte_Rx(PortHandler* ph, uint8_t ID, uint16_t addr, int8_t data, string output)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    packetHandler->write1ByteTxRx(ph, ID, addr, data, &dxl_error);
    if(dxl_error != 0)
        printf("%s", packetHandler->getRxPacketError(dxl_error));
    else
        printf("[ID:%03d] %s success", ID, output.c_str());
}

void motion_physical::read_Byte_Rx(PortHandler* ph, uint8_t ID, uint16_t addr, uint8_t* data, string output)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = packetHandler->read1ByteTxRx(ph, ID, addr, data, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
        ROS_INFO("[ID:%03d] %s: %d", ID, output.c_str(), *data);
    } else {
        ROS_INFO("[ID:%03d] Failed to %s! Result: %d", ID, output.c_str(), dxl_comm_result);
    }
}

void motion_physical::read_4Byte_Rx(PortHandler* ph, uint8_t ID, uint16_t addr, uint32_t* data, string output)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = packetHandler->read4ByteTxRx(ph, ID, addr, data, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
        ROS_INFO("[ID:%03d] %s: %d", ID, output.c_str(), *data);
    } else {
        ROS_INFO("[ID:%03d] Failed to %s! Result: %d", ID, output.c_str(), dxl_comm_result);
    }
}

void motion_physical::SetOperatorDriveMode(PortHandler* ph, uint8_t ID, uint16_t addr, int8_t mode, string output)
{
    write_Byte_Rx(ph, ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, "disable Dynamixel Torque");
    write_Byte_Rx(ph, ID, addr, mode, output);
    write_Byte_Rx(ph, ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, "Enable Dynamixel Torque");
}

//初始化定时器
ros::Timer motion_physical::initializeTimerDef()
{
    ros::param::get("/joints_number", joints_number);
    ros::param::get("arms_number", arms_number);
    ros::param::get("BAUDRATE",BAUDRATE);
    ros::param::get("control_cycle",control_cycle);
    ros::param::get("Gripper_with_current", Gripper_with_current);

    return nh.createTimer(ros::Duration(control_cycle), &motion_physical::Timer_callback, this);
}

//初始化ArmDef类型结构体
motion_physical::ArmDef motion_physical::initializeArmDef(const std::string& paramName)
{
    vector<int> ID;
    string str = "";
    ros::param::get(paramName + "/DXL_ID", ID);
    ros::param::get(paramName + "/DEVICE_NAME", str);
    vector<int> zero(joints_number,0);
    _pid pid;
    
    ArmDef arm =
    {
        str.c_str(), ID, zero, zero, zero, pid,
        PortHandler::getPortHandler(arm.DEVICE_NAME),
        GroupSyncWrite(arm.portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_POSITION),
        GroupSyncRead(arm.portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_POSITION),
        GroupSyncWrite(arm.portHandler, packetHandler, ADDR_GOAL_VELOCITY, LEN_VELOCITY),
        GroupSyncRead(arm.portHandler, packetHandler, ADDR_PRESENT_VELOCITY, LEN_VELOCITY),
        GroupSyncWrite(arm.portHandler, packetHandler, ADDR_GOAL_CURRENT, LEN_CURRENT),
        GroupSyncRead(arm.portHandler, packetHandler, ADDR_PRESENT_CURRENT, LEN_CURRENT)
    };

    return arm;
}

// 功能：将组写电机位置功能打包
// str: "position" "velocity" 默认为"position"
void motion_physical::dxl_tx(ArmDef& Arm, const vector<int32_t> &goal, const string str)
{
    uint8_t param_goal[4];
    int dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    GroupSyncWrite* groupSyncWrite = &Arm.groupSyncWritePosition;
    if(str == "velocity")
        GroupSyncWrite* groupSyncWrite = &Arm.groupSyncWriteVelocity;

    for(size_t i=0; i<joints_number; i++)
    {
        param_goal[0] = DXL_LOBYTE(DXL_LOWORD(goal[i]));
        param_goal[1] = DXL_HIBYTE(DXL_LOWORD(goal[i]));
        param_goal[2] = DXL_LOBYTE(DXL_HIWORD(goal[i]));
        param_goal[3] = DXL_HIBYTE(DXL_HIWORD(goal[i]));

        dxl_addparam_result = groupSyncWrite->addParam(Arm.DXL_ID[i], param_goal);
        if (dxl_addparam_result != true) {
            ROS_ERROR( "Failed to addparam to groupSyncWrite%s for Dynamixel ID %d", str.c_str(), Arm.DXL_ID[i]);
        }
    }

    dxl_comm_result = groupSyncWrite->txPacket();
    if (dxl_comm_result == COMM_SUCCESS){
        // ROS_INFO("set%s success", str.c_str());
    } else {
        ROS_ERROR("Failed to set %s! Result: %d", str.c_str(), dxl_comm_result);
    }

    groupSyncWrite->clearParam();
}

// 功能：将组写电流功能打包
void motion_physical::dxl_tx_cur(ArmDef& Arm, const vector<int16_t> &goal)
{
    uint8_t param_goal[2];
    int dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    for(size_t i=0; i<joints_number; i++)
    {
        param_goal[0] = DXL_LOBYTE(goal[i]);
        param_goal[1] = DXL_HIBYTE(goal[i]);

        dxl_addparam_result = Arm.groupSyncWriteCurrent.addParam(Arm.DXL_ID[i], param_goal);
        if (dxl_addparam_result != true) {
            ROS_ERROR( "Failed to addparam to groupSyncWriteCurrent for Dynamixel ID %d", Arm.DXL_ID[i]);
        }
    }

    dxl_comm_result = Arm.groupSyncWriteCurrent.txPacket();
    if (dxl_comm_result == COMM_SUCCESS){
        ROS_INFO("setCurrent success");
    } else {
        ROS_ERROR("Failed to set Current! Result: %d", dxl_comm_result);
    }

    Arm.groupSyncWriteCurrent.clearParam();
}

// 接收当前位置
// 功能：将组读当前位置功能打包
// str: "position" "velocity" "current" 默认为"position"
void motion_physical::dxl_txRx(ArmDef& Arm, string str)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    int32_t value = 0;

    GroupSyncRead* groupSyncRead = &Arm.groupSyncReadPosition;
    vector<int>* present_value = &Arm.present_position;
    uint16_t addr = ADDR_PRESENT_POSITION;
    uint8_t len_addr = LEN_POSITION;
    if(str == "velocity")
    {
        groupSyncRead = &Arm.groupSyncReadVelocity;
        present_value = &Arm.present_velocity;
        addr = ADDR_PRESENT_VELOCITY;
        len_addr = LEN_VELOCITY;
    }
    else if(str == "current")
    {
        groupSyncRead = &Arm.groupSyncReadCurrent;
        present_value = &Arm.present_current;
        addr = ADDR_PRESENT_CURRENT;
        len_addr = LEN_CURRENT;
    }

    dxl_comm_result = groupSyncRead->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("Failed to get %s! Result: %d", str.c_str(), dxl_comm_result);
        groupSyncRead->clearParam();
    }
    else
    {
        present_value->clear();
        for(size_t i=0;i<Arm.DXL_ID.size();i++)
        {
            value = groupSyncRead->getData(Arm.DXL_ID[i], addr, len_addr);
            if (str == "current" && value > INT16_MAX)
                value = static_cast<int16_t>(value);
            present_value->push_back(value);
            // ROS_INFO("[ID:%03d] get%s : [%s:%d]", Arm.DXL_ID[i], str.c_str(), str.c_str(), (*present_value)[i]);
        }
    }
}

// 机械臂回到初始位置
// 功能：将机械臂回零功能打包
void motion_physical::homing(ArmDef& Arm)
{
    vector<int32_t> zero(joints_number, 0);
    uint8_t dxl_error = 0;
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

    for(size_t i=0; i<joints_number; i++)
    {
        // ping
        packetHandler->ping(Arm.portHandler, Arm.DXL_ID[i], (uint16_t*)&dxl_model_number, &dxl_error);
        if(dxl_error != 0)
            printf("%s", packetHandler->getRxPacketError(dxl_error));
        else
            printf("[ID:%03d] ping Succeeded. Dynamixel model number : %d",Arm.DXL_ID[i], dxl_model_number);

        SetOperatorDriveMode(Arm.portHandler, Arm.DXL_ID[i], ADDR_OPERATOR_MODE, POSITION_MODE, "set operator mode to position");
        SetOperatorDriveMode(Arm.portHandler, Arm.DXL_ID[i], ADDR_DRIVE_MODE, PROFILE_ENABLE, "enable trajectory profile");

        // add groupSyncRead storage
        dxl_addparam_result = Arm.groupSyncReadPosition.addParam(Arm.DXL_ID[i]);
        if (dxl_addparam_result != true)
        {
            ROS_ERROR("Failed to addparam to groupSyncReadPosition for Dynamixel ID %d", Arm.DXL_ID[i]);
        }
        dxl_addparam_result = Arm.groupSyncReadVelocity.addParam(Arm.DXL_ID[i]);
        if (dxl_addparam_result != true)
        {
            ROS_ERROR("Failed to addparam to groupSyncReadVelocity for Dynamixel ID %d", Arm.DXL_ID[i]);
        }
        dxl_addparam_result = Arm.groupSyncReadCurrent.addParam(Arm.DXL_ID[i]);
        if (dxl_addparam_result != true)
        {
            ROS_ERROR("Failed to addparam to groupSyncReadCurrent for Dynamixel ID %d", Arm.DXL_ID[i]);
        }
    }
    
    dxl_tx(Arm, zero, "position");

    while(1)
    {
        uint8_t move_status = 0;
        uint8_t num = 0;
        for(size_t i=0;i<joints_number;i++)
        {
            read_Byte_Rx(Arm.portHandler, Arm.DXL_ID[i], ADDR_MOVE_STATUS, &move_status, "read move status");
            if(move_status) num++;
        }
        if(num == joints_number)
            break;
    }
}

// 配置从机械臂的电机并使其回零
void motion_physical::config_slave_dxl(ArmDef& Arm)
{
    homing(Arm);  // Return to zero

    if(Gripper_with_current)
        SetOperatorDriveMode(Arm.portHandler, Arm.DXL_ID[joints_number - 1], ADDR_OPERATOR_MODE, CURRENT_MODE, "set operator mode to current");
    for(size_t i=0; i<joints_number; i++)
    {
        SetOperatorDriveMode(Arm.portHandler, Arm.DXL_ID[i], ADDR_DRIVE_MODE, PROFILE_DISABLE, "Disable trajectory profile");
    }
}

// 配置主机械臂的电机并使其回零
void motion_physical::config_master_dxl(ArmDef& Arm)
{
    // Return to zero
    homing(Arm);

    for(size_t i=0; i<joints_number; i++)
    {
        SetOperatorDriveMode(Arm.portHandler, Arm.DXL_ID[i], ADDR_OPERATOR_MODE, CURRENT_MODE, "set operator mode to current");
        write_Byte_Rx(Arm.portHandler, Arm.DXL_ID[i], ADDR_GOAL_CURRENT, 0, "set goal cerrnt to 0");
    }
}

// 关节状态发布
void motion_physical::joints_state_publish(ArmDef& Arm, string robot_ref)
{
    sensor_msgs::JointState JointState;

    JointState.header.stamp = ros::Time::now();
    
    for(size_t i=0; i<joints_number; i++)
    {
        string str = to_string(i+1);
        JointState.name.push_back("link" + robot_ref + "_" + str + "_joint");

        JointState.position.push_back(Arm.present_position[i]/pow(2,15) * M_PI * 2);
    }
    joints_state_pub.publish(JointState);
}

// 进行夹爪的位置环pid计算
double motion_physical::Gripper_pid_realize(_pid *pid, int actual_val)
{
	/*计算目标值与实际值的误差*/
    pid->err=pid->target_value-actual_val;

    if(abs(pid->err) <= DXL_MOVING_STATUS_THRESHOLD) pid->err = 0;

    pid->integral += pid->err;    // 误差累积
		
    pid->integral = (pid->integral > 50000) ? 50000 : ((pid->integral < -50000) ? -50000 : pid->integral);

	/*PID算法实现*/
    pid->output_value = pid->Kp * pid->err + pid->Ki * pid->integral + pid->Kd * (pid->err - pid->last_err);
    
    if(pid->output_value>=0) pid->output_value += 35;
    else pid->output_value -= 35;

    /* 输出限幅 */
    pid->output_value = (pid->output_value > pid->output_limit) ? pid->output_limit :\
    (pid->output_value < -pid->output_limit) ? -pid->output_limit : pid->output_value;
  
	/*误差传递*/
    pid->last_err = pid->err;
    
	/*返回当前实际值*/
    return pid->output_value;
}

// 使电机按照指定电流到达指定位置
void motion_physical::SetGripperPositionWithCurrent(ArmDef& Arm, int target_position, uint16_t current)
{
    uint8_t dxl_error = 0;

    Arm.Gripper_pid.target_value = Arm.Gripper_pid.velocity_coefficient * (target_position - Arm.present_position[joints_number-1]);
    Arm.Gripper_pid.output_limit = current;
    if(abs(target_position - Arm.present_position[joints_number-1]) <= DXL_MOVING_STATUS_THRESHOLD)
    {
        Arm.Gripper_pid.target_value = 0;
    }
    int target_current = Gripper_pid_realize(&Arm.Gripper_pid, Arm.present_velocity[joints_number-1]);
    
    packetHandler->write4ByteTxRx(Arm.portHandler, Arm.DXL_ID[joints_number-1], ADDR_GOAL_CURRENT, target_current, &dxl_error);
    if(dxl_error != 0)
        printf("%s", packetHandler->getRxPacketError(dxl_error));
    else
        printf("[ID:%03d] target_current:%d\n", Arm.DXL_ID[joints_number-1], target_current);
}

// 读取电机状态
void motion_physical::statusRead()
{
    dxl_txRx(MasterDxl, "position");
    dxl_txRx(SlaveDxl, "position");
    joints_state_publish(MasterDxl, "1");
    joints_state_publish(SlaveDxl, "2");

    read_4Byte_Rx(MasterDxl.portHandler, MasterDxl.DXL_ID[joints_number-1], ADDR_PRESENT_VELOCITY, (uint32_t*)&MasterDxl.present_velocity[joints_number-1], "get velocity");
    read_4Byte_Rx(SlaveDxl.portHandler, SlaveDxl.DXL_ID[joints_number-1], ADDR_PRESENT_VELOCITY, (uint32_t*)&SlaveDxl.present_velocity[joints_number-1], "get velocity");
}

// 下发从电机当前位置到stm32中
void motion_physical::statusWrite()
{
    if(Gripper_with_current)
        SetGripperPositionWithCurrent(SlaveDxl, MasterDxl.present_position[joints_number-1], current_limit);
    dxl_tx(SlaveDxl, MasterDxl.present_position);
}

// 回调函数，准时读取和发送位置
void motion_physical::Timer_callback(const ros::TimerEvent& event)
{
    statusRead();
    statusWrite();
}

void motion_physical::dyn_cb(follow_control::dynamic_paramConfig& config, uint32_t level)
{
    MasterDxl.Gripper_pid.velocity_coefficient = config.Gripper_pid_velocity_coefficient;
    MasterDxl.Gripper_pid.Kp = config.Gripper_pid_Kp;
    MasterDxl.Gripper_pid.Ki = config.Gripper_pid_Ki;
    MasterDxl.Gripper_pid.Kd = config.Gripper_pid_Kd;
    SlaveDxl.Gripper_pid.velocity_coefficient = config.Gripper_pid_velocity_coefficient;
    SlaveDxl.Gripper_pid.Kp = config.Gripper_pid_Kp;
    SlaveDxl.Gripper_pid.Ki = config.Gripper_pid_Ki;
    SlaveDxl.Gripper_pid.Kd = config.Gripper_pid_Kd;
    current_limit  = config.current_limit;
}