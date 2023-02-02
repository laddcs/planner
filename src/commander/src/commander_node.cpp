#include <commander/commander.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "commander_node");

    ros::NodeHandle nh("");
    ros::NodeHandle control_nh("control");

    ros::CallbackQueue control_queue;
    control_nh.setCallbackQueue(&control_queue);

    commander* C = new commander(nh, control_nh);

    std::thread spinner_control_queue([&control_queue]()
    {
        ros::SingleThreadedSpinner control_spinner;
        control_spinner.spin(&control_queue);
    });

    ros::spin();
    spinner_control_queue.join();
    return 0;
}