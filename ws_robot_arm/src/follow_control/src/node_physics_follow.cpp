#include "../include/physics_follow.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "motion_follow_physical");
    ros::NodeHandle nh;

    if (argc <= 1)
    {
        cout<< "请输入机械臂对的名称" << endl;
        return 0;
    }

    motion_physical motion_physical(nh, argv[1]);

    ros::spin();

    return 0;
}