#include "model/motion_follow.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "motion_follow");
    ros::NodeHandle nh;

    motion_simulation motion_simulation;
    while(ros::ok())
    {

        ros::spinOnce();
    }

    return 0;
}
