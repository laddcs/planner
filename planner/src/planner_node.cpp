#include <planner/planner.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    planner* P = new planner(nh, nh_private);

    ros::spin();
    return 0;
}