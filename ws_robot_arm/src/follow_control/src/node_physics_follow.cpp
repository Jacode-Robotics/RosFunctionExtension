#include "../include/physics_follow.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "motion_follow_physical");
    ros::NodeHandle nh;

    if (argc <= 1)
    {
        cout<< "Please enter the name of this pair of robotic arms" << endl;
        return 0;
    }

    // Argv [1] is the name of the robotic arm
    motion_physical motion_physical(nh, argv[1]);

    ros::spin();

    return 0;
}