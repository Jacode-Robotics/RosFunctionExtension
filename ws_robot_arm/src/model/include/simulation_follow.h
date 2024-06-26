#ifndef __SIMULATION_FOLLOW_H
#define __SIMULATION_FOLLOW_H

    #include <ros/ros.h>
    #include "sensor_msgs/JointState.h"
    #include "trajectory_msgs/JointTrajectory.h"

    #define dxl_num         7           //单机械臂电机关节数量

    // simulation class
    class motion_simulation
    {
        public:
            motion_simulation()
            {
                master_goal_positions_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/master_motion_controller/command", 100);
                slave_goal_positions_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/slave_motion_controller/command", 100);
                master_present_positions_sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, &motion_simulation::MasterPresentPositionsRead,this);
            }

        private:
            ros::NodeHandle nh;
            ros::Publisher master_goal_positions_pub;        // master arms motors goal position publish
            ros::Subscriber master_present_positions_sub;    // master arms motors present position subscribe
            ros::Publisher slave_goal_positions_pub;         // slave arms motors goal position publish

            // publish slave arm motors goal position
            void MasterPresentPositionsRead(const sensor_msgs::JointState::ConstPtr &jointstate);
    };

#endif