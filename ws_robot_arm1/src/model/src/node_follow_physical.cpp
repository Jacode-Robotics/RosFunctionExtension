#include "model/motion_follow.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "motion_follow_physical");
    ros::NodeHandle nh;

    // 实例化控制类
    motion_physical motion_physical(nh);

    ros::spin();

    return 0;
}