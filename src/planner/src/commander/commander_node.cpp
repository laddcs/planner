#include "planner/commander.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "commander_node");
    ros::NodeHandle nh("");

    commander* C = new commander(nh);

    ros::spin();
    return 0;
}