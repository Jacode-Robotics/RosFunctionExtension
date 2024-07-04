#include "../include/physics_follow.h"

motion_physical::motion_physical(ros::NodeHandle& nodehandle, string ArmPairName):
    nh(nodehandle),
    timer( initializeTimerDef(ArmPairName) ),
    MasterDxl( initializeArmDef("MasterDxl", ArmPairName) ),
    SlaveDxl( initializeArmDef("SlaveDxl", ArmPairName) )
{
    // Configure to current mode
    config_dxl(MasterDxl);
    config_dxl(SlaveDxl);

    cbType = boost::bind(&motion_physical::dyn_cb, this, _1, _2);
    server.setCallback(cbType);
}

// delay ms
void delay_ms(int milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

// Write a byte to the protocol table
void motion_physical::write_Byte_Rx(PortHandler* ph, uint8_t ID, uint16_t addr, int8_t data, string output)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    packetHandler->write1ByteTxRx(ph, ID, addr, data, &dxl_error);
    if(dxl_error != 0)
        printf("%s", packetHandler->getRxPacketError(dxl_error));
    else
        printf("[ID:%03d] %s success\n", ID, output.c_str());
}

// read a byte from the protocol table
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

// read 4 byte from the protocol table
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

// Set motor operation or drive mode
// addr: ADDR_OPERATOR_MODE or ADDR_DRIVE_MODE
// mode: Reference Protocol Table
void motion_physical::SetOperatorDriveMode(PortHandler* ph, uint8_t ID, uint16_t addr, int8_t mode, string output)
{
    write_Byte_Rx(ph, ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, "disable Dynamixel Torque");
    write_Byte_Rx(ph, ID, addr, mode, output);
    write_Byte_Rx(ph, ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, "Enable Dynamixel Torque");
}

// Initialize the timer and load parameters
ros::Timer motion_physical::initializeTimerDef(string ArmPairName)
{
    ros::param::get(ArmPairName + "/joints_number", joints_number);
    ros::param::get(ArmPairName + "/BAUDRATE",BAUDRATE);
    ros::param::get(ArmPairName + "/control_cycle",control_cycle);
    ros::param::get(ArmPairName + "/Record_trajectory", Record_trajectory);
    ros::param::get(ArmPairName + "/Reproduction_trajectory", Reproduction_trajectory);
    
    return nh.createTimer(ros::Duration(control_cycle), &motion_physical::Timer_callback, this);
}

// Initialize ArmDef type structure
motion_physical::ArmDef motion_physical::initializeArmDef(const std::string& paramName, string ArmPairName)
{
    vector<int> ID;
    string str = "";
    static vector<float> joint_vel_ratio;
    static vector<float> joint_pid_Kp;
    static vector<float> joint_pid_Ki;
    static vector<float> joint_pid_Kd;
    static vector<_pid> v_pid;
    v_pid.resize(joints_number);
    bool is_first = true;
    
    if(is_first)
    {
        ros::param::get(ArmPairName + "/traj_file_path", traj_file_path);
        ros::param::get(ArmPairName + "/arm_number", arm_number);
        ros::param::get(ArmPairName + "/joint_current_limit", current_limit);
        
        ros::param::get(ArmPairName + "/joint_vel_ratio", joint_vel_ratio);
        ros::param::get(ArmPairName + "/joint_pid_Kp", joint_pid_Kp);
        ros::param::get(ArmPairName + "/joint_pid_Ki", joint_pid_Ki);
        ros::param::get(ArmPairName + "/joint_pid_Kd", joint_pid_Kd);

        is_first = false;
    }
    
    ros::param::get(ArmPairName + "/" + paramName + "/DXL_ID", ID);
    ros::param::get(ArmPairName + "/" + paramName + "/DEVICE_NAME", str);
    for (size_t i = 0; i < joints_number; i++)
    {
        v_pid[i].vel_ratio = joint_vel_ratio[i];
        v_pid[i].Kp = joint_pid_Kp[i];
        v_pid[i].Ki = joint_pid_Ki[i];
        v_pid[i].Kd = joint_pid_Kd[i];
    }
    vector<int> zero(joints_number,0);
    
    ArmDef arm =
    {
        str.c_str(), ID, zero, zero, *(vector<int16_t>*)&zero, v_pid,
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

// Function: Package the position or speed function of the group writing motor
// goal:     target position or target velocity
// str:      "position" "velocity",Default to"position"
void motion_physical::dxl_tx(ArmDef& Arm, const vector<int32_t> &goal, const string str)
{
    uint8_t param_goal[4];
    int dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    // Default to"position"
    GroupSyncWrite* groupSyncWrite = &Arm.groupSyncWritePosition;
    if(str == "velocity")
        GroupSyncWrite* groupSyncWrite = &Arm.groupSyncWriteVelocity;

    // Add data to the group write buffer
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

    // Group write message sending
    dxl_comm_result = groupSyncWrite->txPacket();
    if (dxl_comm_result == COMM_SUCCESS){
        ROS_INFO("set%s success", str.c_str());
    } else {
        ROS_ERROR("Failed to set %s! Result: %d", str.c_str(), dxl_comm_result);
    }

    // Clear group write buffer
    groupSyncWrite->clearParam();
}

// Function: Package the group write current function
// goal:     target current
void motion_physical::dxl_tx_cur(ArmDef& Arm, const vector<int16_t> &goal)
{
    uint8_t param_goal[2];
    int dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    // Add data to the group write buffer
    for(size_t i=0; i<joints_number; i++)
    {
        param_goal[0] = DXL_LOBYTE(goal[i]);
        param_goal[1] = DXL_HIBYTE(goal[i]);

        dxl_addparam_result = Arm.groupSyncWriteCurrent.addParam(Arm.DXL_ID[i], param_goal);
        if (dxl_addparam_result != true) {
            ROS_ERROR( "Failed to addparam to groupSyncWriteCurrent for Dynamixel ID %d", Arm.DXL_ID[i]);
        }
    }

    // Group write message sending
    dxl_comm_result = Arm.groupSyncWriteCurrent.txPacket();
    if (dxl_comm_result == COMM_SUCCESS){
        ROS_INFO("setCurrent success");
    } else {
        ROS_ERROR("Failed to set Current! Result: %d", dxl_comm_result);
    }

    // Clear group write buffer
    Arm.groupSyncWriteCurrent.clearParam();
}

// Function: Package read present status function
// str: "position" "velocity" "current" 默认为"position"
void motion_physical::dxl_txRx(ArmDef& Arm, string str)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    int32_t value = 0;

    // Default to"position"
    GroupSyncRead* groupSyncRead = &Arm.groupSyncReadPosition;
    vector<int32_t>* present_value = &Arm.present_position;
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
        present_value = (vector<int32_t>*)&Arm.present_current;
        addr = ADDR_PRESENT_CURRENT;
        len_addr = LEN_CURRENT;
    }

    // Group read message sending
    dxl_comm_result = groupSyncRead->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("Failed to get %s! Result: %d", str.c_str(), dxl_comm_result);
        groupSyncRead->clearParam();
    }
    else
    {
        present_value->clear();
        for(size_t i=0;i<joints_number;i++)
        {
            // Get data from read buffer 
            value = groupSyncRead->getData(Arm.DXL_ID[i], addr, len_addr);
            if (str == "current" && value > INT16_MAX)
                value = static_cast<int16_t>(value);
            present_value->push_back(value);
            // ROS_INFO("[ID:%03d] get%s : [%s:%d]", Arm.DXL_ID[i], str.c_str(), str.c_str(), (*present_value)[i]);
        }
    }
}

// Return the robotic arm to its initial position
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
    
    // The robotic arm returns to the zero position
    dxl_tx(Arm, zero, "position");

    // Determine whether the robotic arm has reached the target position
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

// Configure the motor of the robotic arm and return it to zero position
void motion_physical::config_dxl(ArmDef& Arm)
{
    // Return to zero position
    homing(Arm);

    // Set motor working mode and operating mode
    for(size_t i=0; i<joints_number; i++)
    {
        SetOperatorDriveMode(Arm.portHandler, Arm.DXL_ID[i], ADDR_OPERATOR_MODE, CURRENT_MODE, "set operator mode to current");
        write_Byte_Rx(Arm.portHandler, Arm.DXL_ID[i], ADDR_GOAL_CURRENT, 0, "set goal cerrnt to 0");
    }
}

// Joints status publish
// robot_ref: Robot arm serial number
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

// Perform speed loop PID calculation for gripper
// pid: The corresponding PID structure of the robotic arm
// actual_val: actual value
double motion_physical::vel_pid_realize(_pid *pid, int actual_val)
{
	/* Calculate the error between the target value and the actual value */
    pid->err=pid->target_value-actual_val;

    /* Speed error */
    if(abs(pid->err) <= DXL_MOVING_STATUS_THRESHOLD)
        pid->err = 0;

    /* integral */
    pid->integral += pid->err;
	
    /* Integral limiting amplitude */
    pid->integral = (pid->integral > 100000) ? 100000 : ((pid->integral < -100000) ? -100000 : pid->integral);

	/* Implementation of PID algorithm */
    pid->output_value = pid->Kp * pid->err + pid->Ki * pid->integral + pid->Kd * (pid->err - pid->last_err);

    /* Output limiting */
    pid->output_value = (pid->output_value > pid->output_limit) ? pid->output_limit :\
    (pid->output_value < -pid->output_limit) ? -pid->output_limit : pid->output_value;
  
	/* Error transmission */
    pid->last_err = pid->err;

    return pid->output_value;
}

// Make the motor reach the specified position according to the specified current
int16_t motion_physical::SetPositionWithCurrent(ArmDef& Arm, uint8_t joint_num, int target_position, uint16_t current)
{
    uint8_t dxl_error = 0;

    // Convert positional deviation to target speed
    Arm.vel_pid[joint_num].target_value = Arm.vel_pid[joint_num].vel_ratio * (target_position - Arm.present_position[joint_num]);
    
    Arm.vel_pid[joint_num].output_limit = current;

    // position error
    if(abs(target_position - Arm.present_position[joint_num]) <= DXL_MOVING_STATUS_THRESHOLD)
    {
        Arm.vel_pid[joint_num].target_value = 0;
    }
    // Calculate current
    return vel_pid_realize(&Arm.vel_pid[joint_num], Arm.present_velocity[joint_num]);
}

// Record the trajectory of the robotic arm
void motion_physical::Record_traj(void)
{
    static bool is_first = true;

    if(is_first)
    {
        // Delete corresponding trajectory file
        for(size_t i=0;i<joints_number;i++)
        {
            // Register file stream
            string str = to_string(i+1);
            string filename = traj_file_path + "/joint" + str + "_pos.traj";
            ifstream file(filename);
            
            // Determine if the file exists and delete it
            if(file.good())
            {
                int result = std::remove(filename.c_str());
                if (result != 0) {
                    perror("Error deleting file"); // 输出错误信息，如果删除失败
                } else {
                    printf("File successfully deleted\n");
                }
            }
        }
        is_first = false;
    }
    // Record trajectory
    for(size_t i=0;i<joints_number;i++)
    {
        string str = to_string(i+1);
        ofstream outFile(traj_file_path + "/joint" + str + "_pos.traj", ios::app);
        if (outFile.is_open()) {
            outFile << setw(traj_point_len) << setfill(' ') << MasterDxl.present_position[i] << endl;
            outFile.close();       // 关闭文件流
        }
    }
}

// Read motor status
void motion_physical::statusRead(void)
{
    dxl_txRx(MasterDxl, "position");
    dxl_txRx(SlaveDxl, "position");
    joints_state_publish(MasterDxl, arm_number[0]);
    joints_state_publish(SlaveDxl, arm_number[1]);

    dxl_txRx(MasterDxl, "velocity");
    dxl_txRx(SlaveDxl, "velocity");

    if(Record_trajectory)
    {
        Record_traj();
    }
}

// Reproduce the trajectory recorded last time
void motion_physical::Follow_TrajFile(void)
{
    static vector<int32_t> file_cursor(joints_number, 0);
    vector<int> target_position(joints_number, 0);
    static vector<bool> is_first(joints_number, true);
    static vector<streampos> file_size(joints_number, 0);
    vector<int16_t> goal_current(joints_number, 0);

    for(size_t i=0;i<joints_number;i++)
    {
        // Open file stream
        string str = to_string(i+1);
        string filename = (traj_file_path + "/joint" + str + "_pos.traj");
        ifstream inFile(filename);

        // Get file size
        if(is_first[i])
        {
            inFile.seekg(0, ios::end);
            file_size[i] = inFile.tellg();

            is_first[i] = false;
        }

        // Reading data from a file
        inFile.seekg(abs(file_cursor[i]), ios::beg);
        string value;
        inFile >> value;
        target_position[i] = stoi(value);
        inFile.close();

        // Set cursor position
        file_cursor[i] += traj_point_len+1;
        if(abs(file_cursor[i])>=file_size[i] - traj_point_len-1) file_cursor[i] = -file_cursor[i];
    }
    cout << target_position[0] << " " << target_position[1] << " " << target_position[2] << " " << target_position[3] << " " << target_position[4] << " " << target_position[5] << endl;
    
    // Calculate master robotic arm current
    for(size_t i=0;i<joints_number;i++)
    {
        goal_current[i] = SetPositionWithCurrent(MasterDxl, i, target_position[i], current_limit[i]);
    }
    // Drive master robotic arm
    dxl_tx_cur(MasterDxl, goal_current);
}

// Send motor control target
void motion_physical::statusWrite(void)
{
     // Reading trajectories
    if(Reproduction_trajectory)
    {
        Follow_TrajFile();
    }

    vector<int16_t> goal_current(joints_number, 0);
    
    // Calculate Slave robotic arm current
    for(size_t i=0;i<joints_number;i++)
    {
        goal_current[i] = SetPositionWithCurrent(SlaveDxl, i, MasterDxl.present_position[i], current_limit[i]);
    }
    // Drive Slave robotic arm
    dxl_tx_cur(SlaveDxl, goal_current);
}

// Callback function, timely reading and sending status
void motion_physical::Timer_callback(const ros::TimerEvent& event)
{
    statusRead();
    statusWrite();
}

// Dynamic parameter callback
void motion_physical::dyn_cb(follow_control::dynamic_paramConfig& config, uint32_t level)
{
    static uint8_t last_joint = 255;
    static bool is_first = true;
    // Synchronize parameter values displayed in rqt
    if(config.joint != last_joint && !is_first)
    {
        config.joint_vel_ratio = MasterDxl.vel_pid[config.joint].vel_ratio;
        config.joint_pid_Kp = MasterDxl.vel_pid[config.joint].Kp;
        config.joint_pid_Ki = MasterDxl.vel_pid[config.joint].Ki;
        config.joint_pid_Kd = MasterDxl.vel_pid[config.joint].Kd;
        config.joint_current_limit = current_limit[config.joint];

        // Manually republish parameters in the callback function to ensure that the update is received by rqt_reconfigure
        server.setCallback(cbType);
    }

    MasterDxl.vel_pid[config.joint].vel_ratio = config.joint_vel_ratio;
    MasterDxl.vel_pid[config.joint].Kp = config.joint_pid_Kp;
    MasterDxl.vel_pid[config.joint].Ki = config.joint_pid_Ki;
    MasterDxl.vel_pid[config.joint].Kd = config.joint_pid_Kd;
    SlaveDxl.vel_pid[config.joint].vel_ratio = config.joint_vel_ratio;
    SlaveDxl.vel_pid[config.joint].Kp = config.joint_pid_Kp;
    SlaveDxl.vel_pid[config.joint].Ki = config.joint_pid_Ki;
    SlaveDxl.vel_pid[config.joint].Kd = config.joint_pid_Kd;
    current_limit[config.joint] = config.joint_current_limit;

    last_joint = config.joint;
    is_first = false;
}