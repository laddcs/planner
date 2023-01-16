#include <controller/controller.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "commander_node");
    ros::NodeHandle nh("");

    controller* C = new controller(nh);

    ros::spin();
    return 0;
}