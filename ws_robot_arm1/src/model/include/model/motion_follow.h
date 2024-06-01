#ifndef __MOTION_FOLLOW_H
#define __MOTION_FOLLOW_H

    #include <ros/ros.h>
    #include "sensor_msgs/JointState.h"
    #include "trajectory_msgs/JointTrajectory.h"
    #include "dynamixel_sdk/dynamixel_sdk.h"
    #include "vector"
    #include <algorithm>
    #include <chrono>
    #include <thread>
    #include <iostream>
    #include <cmath>
    #include <string>

    using namespace dynamixel;
    using namespace std;

    // Control table address
    #define ADDR_TORQUE_ENABLE          512        // Address of enabling torque
    #define ADDR_PRESENT_POSITION       580       // Address of present position
    #define ADDR_GOAL_POSITION          564       // Address of goal position
    #define ADDR_DRIVE_MODE             10        // Address of drive mode(set motor trajectory profile or not,position base to velocity ro base to time,Motor forward and reverse rotation)
    #define ADDR_OPERATOR_MODE          11        // Address of operator mode(current,velocity,position,Extended position Mode)
    #define ADDR_GOAL_CURRENT           550       // Address of goal current

    #define PROTOCOL_VERSION            2.0       // Default Protocol version of DYNAMIXEL X series.

    #define TORQUE_ENABLE               1         // Value for enabling the torque
    #define TORQUE_DISABLE              0         // Value for disabling the torque
    #define PROFILE_ENABLE              0x0       // Value for enable trajectory profile
    #define PROFILE_DISABLE             0x02      // Value for disable trajectory profile
    #define CURRENT_MODE                0X00      // Value for current mode
    #define DXL_MOVING_STATUS_THRESHOLD 20        // position accuracy

    int BAUDRATE           =         2000000;   // Default Baudrate of DYNAMIXEL X series

    int arms_number = 2;
    int joints_number = 6;
    
    // simulation class
    class motion_simulation
    {
        public:
            motion_simulation()
            {
                master_goal_positions_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/master_motion_controller/command", 100);
                slave_goal_positions_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/slave_motion_controller/command", 100);
                master_present_positions_sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 100, &motion_simulation::MasterPresentPositionsRead,this);
            }

        private:
            ros::NodeHandle nh;
            ros::Publisher master_goal_positions_pub;        // master arms motors goal position publish
            ros::Subscriber master_present_positions_sub;    // master arms motors present position subscribe
            ros::Publisher slave_goal_positions_pub;         // slave arms motors goal position publish

            // publish slave arm motors goal position
            void MasterPresentPositionsRead(const sensor_msgs::JointState::ConstPtr &jointstate);
    };

    

    // physical class
    class motion_physical
    {
        public:
            typedef struct
            {
                vector<int> DXL_ID;           // 电机ID
                const char* DEVICE_NAME;          // 串口号

                PortHandler *portHandler;         // 串口Handler
                GroupSyncWrite groupSyncWrite;    // 组写Handler
                GroupSyncRead groupSyncRead;      // 组读Handler
                vector<int32_t> present_position; // 电机当前位置容器
            }ArmDef;

            motion_physical(ros::NodeHandle& nodehandle);

            void PositionsRead();   // 将电机当前位置读取到present_position容器中
            void PositionsWrite();     // 发送目标位置到stm32中
 
        private:
            PacketHandler *packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);  //协议包Handler

            ArmDef MasterLeftDxl;         // 左边主电机结构体定义
            ArmDef MasterRightDxl;        // 右边主电机结构体定义
            ArmDef SlaveLeftDxl;          // 左边从电机结构体定义
            ArmDef SlaveRightDxl;         // 右边从电机结构体定义

            ros::NodeHandle& nh;
            float control_cycle = 0.01;
            ros::Timer timer;             // 定时器
            ros::Publisher joints_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 100);

            void config_slave_dxl(ArmDef& Arm);        // 配置从电机并使电机回零
            void config_master_dxl(ArmDef& Arm);       // 配置主电机并使电机回零
            void dxl_positions_tx(ArmDef& Arm, const vector<int32_t> &slave_position);   // 发送目标位置
            void dxl_positions_txRx(ArmDef& Arm);      // 读取当前位置
            void homing(ArmDef& Arm);                  // 回零
            void Timer_callback(const ros::TimerEvent& event);
            void joints_state_publish(ArmDef& Arm, string robot_ref);
            ArmDef initializeArmDef(const std::string& paramName);
            ros::Timer initializeTimerDef();
    };

#endif