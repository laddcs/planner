#include <planner/planner.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh("");

    planner* P = new planner(nh);

    ros::spin();
    return 0;
}