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

    #define ADDR_MOVE_STATUS            571       // Address of rotating state(Determine if the target position has been reached)
    
    #define PROTOCOL_VERSION            2.0       // Default Protocol version of DYNAMIXEL X series.

    #define TORQUE_ENABLE               1         // Value for enabling the torque
    #define TORQUE_DISABLE              0         // Value for disabling the torque
    #define PROFILE_ENABLE              0x0       // Value for enable trajectory profile
    #define PROFILE_DISABLE             0x02      // Value for disable trajectory profile
    #define POSITION_MODE               0X03      // Value for position mode
    #define CURRENT_MODE                0X00      // Value for current mode
    #define DXL_MOVING_STATUS_THRESHOLD 20        // position accuracy
    #define DXL_VELOCITY_THRESHOLD      50        // velocity accuracy

    #define TRAJ_POINT_LEN              7         // The length of a trajectory point in a traj file

    // physical class
    class motion_physical
    {
        public:
            typedef struct
            {
                float vel_ratio;    // The ratio of speed to position
                float Kp,Ki,Kd;          	   // Proportional, integral, and differential coefficients
                int integral;          		   // Integral value
                int err;                       // Difference
                int last_err;                  // last Difference
                int target_value;              // target value
                int output_value;              // Output value
                int output_limit;              // Output limitation
            }_pid;

            typedef struct
            {
                const char* DEVICE_NAME;                 // Serial port number
                vector<int> DXL_ID;                      // motor ID
                vector<int32_t> present_position;        // Current position of motor
                vector<int32_t> present_velocity;        // Current speed of motor
                vector<int16_t> present_current;         // Current of motor
                vector<_pid> vel_pid;                // Gripper speed loop PID

                PortHandler *portHandler;                // Serial port Handler
                GroupSyncWrite groupSyncWritePosition;   // Group Write Target Location Handler
                GroupSyncRead groupSyncReadPosition;     // Group read current position Handler
                GroupSyncWrite groupSyncWriteVelocity;   // Group write target speed Handler
                GroupSyncRead groupSyncReadVelocity;     // Group read current speed Handler
                GroupSyncWrite groupSyncWriteCurrent;    // Group write target current Handler
                GroupSyncRead groupSyncReadCurrent;      // Group reading current Handler
                
            }ArmDef;

            motion_physical(ros::NodeHandle& nodehandle, string ArmPairName);

            void statusRead();      // Read motor status
            void statusWrite();     // Send target location to stm32
 
        private:
            ros::NodeHandle& nh;
            ros::Timer timer;
            ros::Publisher joints_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

            // Retrieve from parameter server
            int BAUDRATE;                        // BAUDRATE
            float control_cycle;                 // control cycle
            int joints_number;                   // joints number
            vector<int> current_limit;      // current limit
            bool Record_trajectory;              // Record trajectory or not
            bool Reproduction_trajectory;        // Reproduction trajectory or not
            string traj_file_path;               // trajectory file path
            vector<string> arm_number;

            //Protocol package Handler
            PacketHandler *packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

            ArmDef MasterDxl;         // Master motor structure
            ArmDef SlaveDxl;          // Slave motor structure

            // dynamic parameter
            dynamic_reconfigure::Server<follow_control::dynamic_paramConfig> server;
            dynamic_reconfigure::Server<follow_control::dynamic_paramConfig>::CallbackType cbType;

            inline void write_Byte_Rx(PortHandler* portHandler, uint8_t ID, uint16_t addr, int8_t data, string output);
            inline void read_Byte_Rx(PortHandler* ph, uint8_t ID, uint16_t addr, uint8_t* data, string output);
            inline void read_4Byte_Rx(PortHandler* ph, uint8_t ID, uint16_t addr, uint32_t* data, string output);
            inline void SetOperatorDriveMode(PortHandler* ph, uint8_t ID, uint16_t addr, int8_t mode, string output);
            inline ArmDef initializeArmDef(const std::string& paramName, string ArmPairName);
            inline ros::Timer initializeTimerDef(string ArmPairName);
            inline void dxl_tx(ArmDef& Arm, const vector<int32_t> &goal, const string str = "position");
            inline void dxl_tx_cur(ArmDef& Arm, const vector<int16_t> &goal);
            inline void dxl_txRx(ArmDef& Arm, string str = "position");
            inline void homing(ArmDef& Arm);
            inline void config_dxl(ArmDef& Arm);
            inline void joints_state_publish(ArmDef& Arm, string robot_ref);
            inline double vel_pid_realize(_pid *pid, int actual_val);
            inline int16_t SetPositionWithCurrent(ArmDef& Arm, uint8_t joint_num, int target_position, uint16_t current);
            inline void Follow_TrajFile(void);
            inline void Record_traj(void);
            void dyn_cb(follow_control::dynamic_paramConfig& config, uint32_t level);
            void Timer_callback(const ros::TimerEvent& event);
    };

#endif