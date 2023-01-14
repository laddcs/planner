#include <planner/controller.h>

controller::controller(const ros::NodeHandle &nh) : nh_(nh)
{
    cmd_sub_ = nh_.subscribe("commander/state", 1, &controller::cmd_msg_cb, this);
    setpoint_sub_ = nh_.subscribe("mavros/setpoint_raw/target_local", 1, &controller::setpoint_cb, this);

    setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    controller_timer_ = nh_.createTimer(ros::Duration(0.1), &controller::controller_cb, this);

    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    last_request = ros::Time::now();
}

void controller::cmd_msg_cb(const planner_msgs::CommandState::ConstPtr& msg)
{
    current_cmd_ = *msg;
}

void controller::setpoint_cb(const mavros_msgs::PositionTarget::ConstPtr& msg)
{
    current_px4_setpoint_.header.frame_id = "map";
    current_px4_setpoint_.header.stamp = ros::Time::now();
    current_px4_setpoint_.pose.position = (*msg).position;
    current_px4_setpoint_.pose.orientation = euler2Quat(0, 0, (*msg).yaw);
}

void controller::controller_cb(const ros::TimerEvent& event)
{
    if(current_cmd_.state == planner_msgs::CommandState::PLANNING)
    {
        if((current_cmd_.px4_state != mavros_msgs::State::MODE_PX4_OFFBOARD) && ((ros::Time::now() - last_request) > ros::Duration(5.0)))
        {
            mavros_msgs::SetMode offb_set_mode;
            offb_set_mode.request.custom_mode = "OFFBOARD";

            last_request = ros::Time::now();

            if(set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled!");
            }
        }

        setpoint_pub_.publish(current_px4_setpoint_);
    }
}
