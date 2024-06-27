#ifndef __PHYSICS_FOLLOW_H
#define __PHYSICS_FOLLOW_H

    #include <ros/ros.h>
    #include "sensor_msgs/JointState.h"
    #include "dynamixel_sdk/dynamixel_sdk.h"
    #include "vector"
    #include <algorithm>
    #include <chrono>
    #include <thread>
    #include <iostream>
    #include <cmath>
    #include <string>
    #include <numeric>
    #include <fstream>
    #include "dynamic_reconfigure/server.h"
    #include "follow_control/dynamic_paramConfig.h"
    #include <cstdio> 

    using namespace dynamixel;
    using namespace std;

    // Control table address
    #define ADDR_TORQUE_ENABLE          512       // Address of enabling torque

    #define ADDR_PRESENT_POSITION       580       // Address of present position
    #define ADDR_GOAL_POSITION          564       // Address of goal position
    #define LEN_POSITION                4         // lenght of position

    #define ADDR_PRESENT_VELOCITY       576       // Address of present velocity
    #define ADDR_GOAL_VELOCITY          552       // Address of goal velocity
    #define LEN_VELOCITY                4         // lenght of velocity

    #define ADDR_PRESENT_CURRENT        574       // Address of present current
    #define ADDR_GOAL_CURRENT           550       // Address of goal current
    #define LEN_CURRENT                 2         // lenght of position

    #define ADDR_DRIVE_MODE             10        // Address of drive mode(set motor trajectory profile or not,position base to velocity ro base to time,Motor forward and reverse rotation)
    #define ADDR_OPERATOR_MODE          11        // Address of operator mode(current,velocity,position,Extended position Mode)

    #define ADDR_MOVE_STATUS            571
    
    #define PROTOCOL_VERSION            2.0       // Default Protocol version of DYNAMIXEL X series.

    #define TORQUE_ENABLE               1         // Value for enabling the torque
    #define TORQUE_DISABLE              0         // Value for disabling the torque
    #define PROFILE_ENABLE              0x0       // Value for enable trajectory profile
    #define PROFILE_DISABLE             0x02      // Value for disable trajectory profile
    #define POSITION_MODE               0X03      // Value for current mode
    #define CURRENT_MODE                0X00      // Value for current mode
    #define DXL_MOVING_STATUS_THRESHOLD 20        // position accuracy
    #define DXL_VELOCITY_THRESHOLD      50        // velocity accuracy

    #define traj_point_len              7

    // physical class
    class motion_physical
    {
        public:
            typedef struct
            {
                float velocity_coefficient;
                float Kp,Ki,Kd;          		//比例、积分、微分系数
                int integral;          		//积分值
                int err;
                int last_err;
                int target_value;
                int output_value;
                int output_limit;
            }_pid;

            typedef struct
            {
                const char* DEVICE_NAME;    // 串口号
                vector<int> DXL_ID;         // 电机ID
                vector<int32_t> present_position;        // 电机当前位置
                vector<int32_t> present_velocity;        // 电机当前速度
                vector<int32_t> present_current;         // 电机当前电流
                _pid Gripper_pid;           // 夹爪位置环pid

                PortHandler *portHandler;   // 串口Handler
                GroupSyncWrite groupSyncWritePosition;   // 组写目标位置Handler
                GroupSyncRead groupSyncReadPosition;     // 组读当前位置Handler
                GroupSyncWrite groupSyncWriteVelocity;   // 组写目标速度Handler
                GroupSyncRead groupSyncReadVelocity;     // 组读当前速度Handler
                GroupSyncWrite groupSyncWriteCurrent;    // 组写目标电流Handler
                GroupSyncRead groupSyncReadCurrent;      // 组读当前电流Handler
                
            }ArmDef;

            motion_physical(ros::NodeHandle& nodehandle);

            void statusRead();   // 将电机当前位置读取到present_position容器中
            void statusWrite();     // 发送目标位置到stm32中
 
        private:
            ros::NodeHandle& nh;
            ros::Timer timer;             // 定时器
            ros::Publisher joints_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

            int BAUDRATE;
            float control_cycle;
            int arms_number;
            int joints_number;
            bool Gripper_with_current;
            uint16_t current_limit;
            bool Record_trajectory;
            bool Reproduction_trajectory;
            string traj_file_path;

            PacketHandler *packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);  //协议包Handler

            ArmDef MasterDxl;         // 主电机结构体
            ArmDef SlaveDxl;          // 从电机结构体

            dynamic_reconfigure::Server<follow_control::dynamic_paramConfig> server;                  // 创建服务器对象
            dynamic_reconfigure::Server<follow_control::dynamic_paramConfig>::CallbackType cbType;    // 创建回调对象(使用回调函数，打印修改后的参数)

            inline void write_Byte_Rx(PortHandler* portHandler, uint8_t ID, uint16_t addr, int8_t data, string output);
            inline void read_Byte_Rx(PortHandler* ph, uint8_t ID, uint16_t addr, uint8_t* data, string output);
            inline void read_4Byte_Rx(PortHandler* ph, uint8_t ID, uint16_t addr, uint32_t* data, string output);
            inline void SetOperatorDriveMode(PortHandler* ph, uint8_t ID, uint16_t addr, int8_t mode, string output);
            inline ArmDef initializeArmDef(const std::string& paramName);
            inline ros::Timer initializeTimerDef(void);
            inline void dxl_tx(ArmDef& Arm, const vector<int32_t> &goal, const string str = "position");   // 发送目标位置
            inline void dxl_tx_cur(ArmDef& Arm, const vector<int16_t> &goal);
            inline void dxl_txRx(ArmDef& Arm, string str = "position");                // 读取当前值
            inline void homing(ArmDef& Arm);                  // 回零
            inline void config_slave_dxl(ArmDef& Arm);        // 配置从电机并使电机回零
            inline void config_master_dxl(ArmDef& Arm);       // 配置主电机并使电机回零
            inline void joints_state_publish(ArmDef& Arm, string robot_ref);
            inline double Gripper_pid_realize(_pid *pid, int actual_val);
            inline void SetGripperPositionWithCurrent(ArmDef& Arm, int target_position, uint16_t current);
            inline void Follow_TrajFile(void);
            inline void Record_traj(void);
            void dyn_cb(follow_control::dynamic_paramConfig& config, uint32_t level);
            void Timer_callback(const ros::TimerEvent& event);
    };

#endif