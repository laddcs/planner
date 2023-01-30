#include <controller/controller.h>

controller::controller(const ros::NodeHandle &nh) : nh_(nh)
{
    setpoint_sub_ = nh_.subscribe("mavros/setpoint_raw/target_local", 1, &controller::setpoint_cb, this);
    state_sub_ = nh_.subscribe("mavros/state", 1, &controller::state_cb, this);
    pose_sub_ = nh_.subscribe("mavroslocal_position/pose", 1, &controller::pose_cb, this);

    setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    controller_timer_ = nh_.createTimer(ros::Duration(0.1), &controller::controller_cb, this);

    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    set_commander_client_ = nh_.serviceClient<planner_msgs::SetCommander>("commander/set_commander");
    plan_path_client_ = nh_.serviceClient<planner_msgs::PlanPath>("planner/plan_path");
    set_controller_ = nh_.advertiseService("controller/set_controller", &controller::set_controller_cb, this);

    last_request = ros::Time::now();

    plan_ = false;
    track_ = false;
    tracking_ = false;

    track_idx_ = 0;
}

bool controller::set_controller_cb(planner_msgs::SetController::Request& request, planner_msgs::SetController::Response& response)
{
    if(request.plan)
    {
        start_ = request.start;
        goal_ = request.goal;
        plan_ = request.plan;

        response.success = true;

        return true;
    }

    plan_ = request.plan;
    track_ = request.track;

    if(!track_)
    {
        tracking_ = false;
    }

    response.success = true;

    return false;
}

void controller::setpoint_cb(const mavros_msgs::PositionTarget::ConstPtr& msg)
{
    current_px4_setpoint_.header.frame_id = "map";
    current_px4_setpoint_.header.stamp = ros::Time::now();
    current_px4_setpoint_.pose.position = (*msg).position;
    current_px4_setpoint_.pose.orientation = euler2Quat(0, 0, (*msg).yaw);
}

void controller::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_ = *msg;
}

void controller::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose_ = (*msg).pose;
}

void controller::controller_cb(const ros::TimerEvent& event)
{
    if(plan_)
    {
        // call plan service here
        planner_msgs::PlanPath plan;
        plan.request.start = start_;
        plan.request.goal = goal_;
        if(plan_path_client_.call(plan) && plan.response.success)
        {
            trajectory_ = plan.response.path.plan;

            current_offboard_setpoint_.pose = trajectory_[0].pos;
            current_offboard_setpoint_.header.frame_id = "map";

            planner_msgs::SetCommander set_commander;
            set_commander.request.has_plan = true;
            set_commander.request.track_complete = false;
            if(set_commander_client_.call(set_commander) && set_commander.response.success)
            {
                ROS_INFO("Planning Successful!");
                plan_ = false;
                track_ = true;
            }

        } else 
        {
            ROS_INFO("Planning Failure!");
        }
    }

    if(track_)
    {
        if((current_state_.mode != mavros_msgs::State::MODE_PX4_OFFBOARD))
        {
            if(((ros::Time::now() - last_request) > ros::Duration(5.0)))
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
            return;
        }

        if(!tracking_)
        {
            if(((current_pose_.position.z - trajectory_[0].pos.position.z) > 0.1))
            {
                setpoint_pub_.publish(current_offboard_setpoint_);
                return;
            } else
            {
                tracking_ = true;
                track_start_ = ros::Time::now();
            }
        }

        if(track_idx_ < trajectory_.size())
        {
            if((ros::Time::now() - track_start_) > (trajectory_[track_idx_].target_time.data))
            {
                track_idx_ ++;
                current_offboard_setpoint_.pose = trajectory_[track_idx_].pos;
                current_offboard_setpoint_.header.frame_id = "map";
            }
            setpoint_pub_.publish(current_offboard_setpoint_);
        } else
        {
            tracking_ = false;
            
            planner_msgs::SetCommander set_commander;
            set_commander.request.has_plan = false;
            set_commander.request.track_complete = true;
            if(set_commander_client_.call(set_commander) && set_commander.response.success)
            {
                ROS_INFO("Completed Track!");
                track_ = false;
            }
            setpoint_pub_.publish(current_offboard_setpoint_);
        }
    }
}
