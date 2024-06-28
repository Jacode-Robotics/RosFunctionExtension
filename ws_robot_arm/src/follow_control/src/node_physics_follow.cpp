#include "../include/physics_follow.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "motion_follow_physical");
    ros::NodeHandle nh;

    motion_physical motion_physical(nh);

    ros::spin();

    return 0;
}