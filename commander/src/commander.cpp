#include <commander/commander.h>

commander::commander(const ros::NodeHandle &nh, const ros::NodeHandle &control_nh) : nh_(nh), control_nh_(control_nh)
{
    state_sub_ = nh_.subscribe("mavros/state", 1, &commander::state_cb, this);
    setpoint_sub_ = nh_.subscribe("mavros/setpoint_raw/target_local", 1, &commander::setpoint_cb, this);
    waypoint_sub_ = nh_.subscribe("mavros/mission/waypoints", 1, &commander::waypoint_cb, this);
    home_sub_ = nh_.subscribe("mavros/home_position/home", 1, &commander::home_cb, this);
    pose_sub_ = nh_.subscribe("mavros/local_position/pose", 1, &commander::pose_cb, this);

    setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    status_pub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>("mavros/companion_process/status", 1);

    plan_path_client_ = nh_.serviceClient<planner_msgs::PlanPath>("planner/plan_path");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    waypoint_pull_client_ = nh_.serviceClient<mavros_msgs::WaypointPull>("mavros/mission/pull");

    cmdloop_timer_ = nh_.createTimer(ros::Duration(0.1), &commander::cmdloop_cb, this);
    ctlloop_timer_ = control_nh_.createTimer(ros::Duration(0.1), &commander::ctlloop_cb, this);

    // Tracking Info
    last_request = ros::Time::now();

    // State Flags
    home_set_ = false;
    has_goal_ = false;
    planning_ = false;
    has_plan_ = false;
    tracking_ = false;
    track_complete_ = false;
    cmd_state_ = CMD_STATE::IDLE;
    system_status_.component = mavros_msgs::CompanionProcessStatus::MAV_COMP_ID_OBSTACLE_AVOIDANCE;
    system_status_.state = mavros_msgs::CompanionProcessStatus::MAV_STATE_UNINIT;

    // Coordinate Transform
    earth = GeographicLib::Geocentric(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
}

void commander::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_ = *msg;
}

void commander::home_cb(const mavros_msgs::HomePosition::ConstPtr& msg)
{
    _Float64 lat0, lon0, alt0;
    current_home_ = *msg;

    lat0 = current_home_.geo.latitude;
    lon0 = current_home_.geo.longitude;
    alt0 = current_home_.geo.altitude;

    earth.Forward(lat0, lon0, alt0,
        ecef_origin_(0), ecef_origin_(1), ecef_origin_(2));

    map_origen_ << lat0, lon0, alt0;

    if(!home_set_)
    {
        home_set_ = true;
        ROS_INFO("CMD: Home Set!");
        ROS_INFO("CMD: Home Coordinates at: (%3f, %3f, %3f)", lat0, lon0, alt0);
    }
}

void commander::waypoint_cb(const mavros_msgs::WaypointList::ConstPtr& msg)
{
    ROS_INFO("CMD: Mission Recieved!");
    for(int i = 0; i < (*msg).waypoints.size(); i++)
    {
        if((*msg).waypoints[i].command == mavros_msgs::CommandCode::NAV_WAYPOINT)
        {
            if((current_mission_.x_lat != (*msg).waypoints[i].x_lat) || (current_mission_.y_long != (*msg).waypoints[i].y_long))
            {
                current_mission_ = (*msg).waypoints[i];
                has_goal_ = true;
                ROS_INFO("CMD: Guidance goal set!");
            }
            break;
        }
    }
    if(!tracking_)
    {
        system_status_.state = mavros_msgs::CompanionProcessStatus::MAV_STATE_STANDBY;
    }
}

void commander::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose_ = (*msg).pose;
}

void commander::setpoint_cb(const mavros_msgs::PositionTarget::ConstPtr& msg)
{
    current_px4_setpoint_.pose.position = (*msg).position;
    current_px4_setpoint_.pose.orientation = euler2Quat(0, 0, (*msg).yaw);
    current_px4_setpoint_.header.stamp = ros::Time::now();
    current_px4_setpoint_.header.frame_id = "map";
}

void commander::cmdloop_cb(const ros::TimerEvent &event)
{
    if(cmd_state_ == CMD_STATE::IDLE)
    {        
        if((current_state_.mode == mavros_msgs::State::MODE_PX4_MISSION) && current_state_.armed)
        {
            // If there is no goal set call to update the goal
            if(!has_goal_)
            {
                mavros_msgs::WaypointPull waypoint_pull;
                if(!waypoint_pull_client_.call(waypoint_pull) || !waypoint_pull.response.success) ROS_WARN("CMD: Error Pulling Waypoints!");
                else ROS_INFO("CMD: Pulling new guidance goal!");
            }

            // Overide the mission and transfer back to hold
            mavros_msgs::SetMode plan_set_mode;
            plan_set_mode.request.custom_mode = "AUTO.LOITER";
            if(set_mode_client_.call(plan_set_mode) && plan_set_mode.response.mode_sent)
            {
                // If the transform is valid, and a goal exists -> start the planner
                if(home_set_ && has_goal_)
                {
                    // Transform from QGC geodetic frame to mavros NED frame
                    Eigen::Vector3d map_point;
                    Eigen::Vector3d local_ecef;
                    Eigen::Vector3d goal_point;

                    earth.Forward(current_mission_.x_lat, current_mission_.y_long, current_mission_.z_alt + current_home_.geo.altitude, 
                        map_point(0), map_point(1), map_point(2));

                    local_ecef = map_point - ecef_origin_;

                    goal_point = mavros::ftf::transform_frame_ecef_enu(local_ecef, map_origen_);

                    // Set the Start Pose to the current pose, set the Goal Pose to the waypoint from QGC
                    goal_pose_.position.x = goal_point(0);
                    goal_pose_.position.y = goal_point(1);
                    goal_pose_.position.z = goal_point(2);

                    // If a yaw us avialable in waypoint info then set the goal yaw to it, otherwise set it to current yaw
                    if(!std::isnan(current_mission_.param4)) goal_pose_.orientation = euler2Quat(0, 0, current_mission_.param4*2*M_PI/180);
                    else goal_pose_.orientation = current_pose_.orientation;

                    cmd_state_ = CMD_STATE::PLANNING;
                    has_goal_ = false;
                    planning_ = true;

                    system_status_.state = mavros_msgs::CompanionProcessStatus::MAV_STATE_ACTIVE;
                } else
                {
                    if(!home_set_) ROS_WARN("CMD: Warning! Home Not Set!");
                    else if(!has_goal_) ROS_WARN("CMD: Warning! No Guidance Goal!");
                }
            }
        } else if(((current_state_.mode == mavros_msgs::State::MODE_PX4_MANUAL) || ((current_state_.mode == mavros_msgs::State::MODE_PX4_POSITION)))
            && current_state_.armed)
        {
            ROS_INFO("CMD: RC control enabled.");
            cmd_state_ = CMD_STATE::RC;
        }
    }

    if(cmd_state_ == CMD_STATE::PLANNING)
    {
        if(has_plan_ && current_state_.mode == mavros_msgs::State::MODE_PX4_LOITER)
        {
            ROS_INFO("CMD: Initializing trajectory tracking!");
            has_plan_ = false;
            cmd_state_ = CMD_STATE::TRACK;
        } else if((current_state_.mode == mavros_msgs::State::MODE_PX4_MANUAL) || ((current_state_.mode == mavros_msgs::State::MODE_PX4_POSITION))
            && current_state_.armed)
        {
            ROS_INFO("CMD: RC control enabled, abandoning planning.");
            cmd_state_ = CMD_STATE::RC;
        } else if((current_state_.mode == mavros_msgs::State::MODE_PX4_RTL) || (current_state_.mode == mavros_msgs::State::MODE_PX4_LAND))
        {
            ROS_INFO("CMD: PX4 Autonomous mode enabled, abandoning planning.");
            cmd_state_ = CMD_STATE::IDLE;
        } else if(!planning_ && !has_plan_)
        {
            // Set the companion state to emergency if the planner fails
            system_status_.state = mavros_msgs::CompanionProcessStatus::MAV_STATE_EMERGENCY;
            cmd_state_ = CMD_STATE::IDLE;
        }
    }

    if(cmd_state_ == CMD_STATE::TRACK)
    {
        if(track_complete_)
        {
            track_complete_ = false;
            mavros_msgs::SetMode plan_set_mode;
            plan_set_mode.request.custom_mode = "AUTO.LOITER";
            if(set_mode_client_.call(plan_set_mode) && plan_set_mode.response.mode_sent)
            {
                ROS_INFO("CMD: Return to Idle.");
                cmd_state_ = CMD_STATE::IDLE;
            }
            system_status_.state = mavros_msgs::CompanionProcessStatus::MAV_STATE_STANDBY;
        } else if(((current_state_.mode == mavros_msgs::State::MODE_PX4_MANUAL) || ((current_state_.mode == mavros_msgs::State::MODE_PX4_POSITION)))
            && current_state_.armed)
        {
            ROS_INFO("CMD: RC control enabled, abandoning tracking.");
            tracking_ = false;
            cmd_state_ = CMD_STATE::RC;
        } else if((current_state_.mode == mavros_msgs::State::MODE_PX4_RTL) || (current_state_.mode == mavros_msgs::State::MODE_PX4_LAND))
        {
            ROS_INFO("CMD: PX4 Autonomous mode enabled, abandoning tracking.");
            tracking_ = false;
            cmd_state_ = CMD_STATE::IDLE;
        }
    }

    if(cmd_state_ == CMD_STATE::RC)
    {
        // When the system is in RC let the companion process to critical
        system_status_.state = mavros_msgs::CompanionProcessStatus::MAV_STATE_CRITICAL;
        if(!(current_state_.mode == mavros_msgs::State::MODE_PX4_MANUAL) && !(current_state_.mode == mavros_msgs::State::MODE_PX4_POSITION))
        {
            ROS_INFO("CMD: Entering autonomous mode.");
            cmd_state_ = CMD_STATE::IDLE;
        }
    }

    status_pub_.publish(system_status_);
}

void commander::ctlloop_cb(const ros::TimerEvent &event)
{
    if(current_state_.armed)
    {
        if(planning_)
        {
            ROS_INFO("CMD: Start set to: (%3f, %3f, %3f)", current_pose_.position.x, current_pose_.position.y, extract_yaw(current_pose_.orientation));
            ROS_INFO("CMD: Goal set to: (%3f, %3f, %3f)", goal_pose_.position.x, goal_pose_.position.y, extract_yaw(goal_pose_.orientation));
            ROS_INFO("CMD: Initializing trajectory planning!");

            // Call plan service here
            planner_msgs::PlanPath plan;
            plan.request.start = current_pose_;
            plan.request.goal = goal_pose_;
            if(plan_path_client_.call(plan) && plan.response.success)
            {
                trajectory_ = plan.response.path.plan;

                current_offboard_setpoint_.pose.position = trajectory_[0].pos.position;
                current_offboard_setpoint_.pose.orientation = trajectory_[0].pos.orientation;
                current_offboard_setpoint_.header.stamp = ros::Time::now();
                current_offboard_setpoint_.header.frame_id = "map";

                has_plan_ = true;
                planning_ = false;
                track_idx_ = 0;
            } else 
            {
                ROS_WARN("CMD: Planning Failure!");
                planning_ = false;
            }
        }

        if(cmd_state_ == CMD_STATE::TRACK)
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
                        ROS_INFO("CMD: Offboard enabled!");
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
                return;
            }

            if(track_idx_ < trajectory_.size())
            {
                if((ros::Time::now() - track_start_) > (trajectory_[track_idx_].target_time.data))
                {
                    track_idx_ ++;
                    current_offboard_setpoint_.pose.position = trajectory_[track_idx_].pos.position;
                    current_offboard_setpoint_.pose.orientation = trajectory_[track_idx_].pos.orientation;
                    current_offboard_setpoint_.header.stamp = ros::Time::now();
                    current_offboard_setpoint_.header.frame_id = "map";
                }
                setpoint_pub_.publish(current_offboard_setpoint_);
            } else
            {
                tracking_ = false;
                track_complete_ = true;
            }
        }

        if(!(cmd_state_ == CMD_STATE::PLANNING) && !(cmd_state_ == CMD_STATE::TRACK))
        {
            setpoint_pub_.publish(current_px4_setpoint_);
        }
    }

}