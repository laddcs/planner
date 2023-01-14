#include <planner/controller.h>

controller::controller(const ros::NodeHandle &nh) : nh_(nh)
{
    cmd_sub_ = nh_.subscribe("commander/state", 1, &controller::cmd_msg_cb, this);
}

void controller::cmd_msg_cb(const planner_msgs::CommandState::ConstPtr& msg)
{

}