#include "../include/simulation_follow.h"

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
            JointTrajectory.joint_names[i] = "link2_" + str + "_joint";
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
