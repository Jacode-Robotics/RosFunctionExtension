#include "../include/simulation_follow.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "simulation_follow");
    ros::NodeHandle nh;

    motion_simulation motion_simulation;
    while(ros::ok())
    {

        ros::spinOnce();
    }

    return 0;
}
