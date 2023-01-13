#include <planner/commander.h>

commander::commander(const ros::NodeHandle &nh) : nh_(nh)
{
    state_sub_ = nh_.subscribe("mavros/state", 1, &commander::state_cb, this);
    waypoint_sub_ = nh_.subscribe("mavros/mission/waypoints", 1, &commander::waypoint_cb, this);
    home_sub_ = nh_.subscribe("mavros/home_position/home", 1, &commander::home_cb, this);

    command_pub_ = nh_.advertise<planner_msgs::CommandState>("commander/state", 1);

    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    cmdloop_timer_ = nh_.createTimer(ros::Duration(0.1), &commander::cmdloop_cb, this);

    home_set_ = false;
    has_goal_ = false;
    has_plan_ = false;
    track_complete_ = false;
    cmd_state_ = CMD_STATE::IDLE;

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

    ROS_INFO("Home Coordinates at: (%3f, %3f, %3f)", lat0, lon0, alt0);

    if(!home_set_)
    {
        home_set_ = true;
        ROS_INFO("Home Set!");
    }
}

void commander::waypoint_cb(const mavros_msgs::WaypointList::ConstPtr& msg)
{
    ROS_INFO("Mission Recieved!");
    for(int i = 0; i < (*msg).waypoints.size(); i++)
    {
        if((*msg).waypoints[i].command == mavros_msgs::CommandCode::NAV_WAYPOINT)
        {
            current_mission_ = (*msg).waypoints[i];
            has_goal_ = true;
            ROS_INFO("Guidance goal set!");
            break;
        }
    }
}

void commander::cmdloop_cb(const ros::TimerEvent &event)
{
    planner_msgs::CommandState cmd;
    cmd.has_goal = false;
    cmd.track_target = false;

    if(cmd_state_ == CMD_STATE::IDLE)
    {
        cmd.state = planner_msgs::CommandState::IDLE;

        if((current_state_.mode == mavros_msgs::State::MODE_PX4_MISSION) && current_state_.armed)
        {
            mavros_msgs::SetMode plan_set_mode;
            plan_set_mode.request.custom_mode = "AUTO.LOITER";
            if(set_mode_client_.call(plan_set_mode) && plan_set_mode.response.mode_sent && has_goal_)
            {
                if(home_set_)
                {
                    ROS_INFO("Initializing trajectory planning!");
                    cmd_state_ = CMD_STATE::PLANNING;

                    // Transform from QGC geodetic frame to mavros NED frame
                    Eigen::Vector3d map_point;
                    Eigen::Vector3d local_ecef;
                    Eigen::Vector3d goal_point;

                    earth.Forward(current_mission_.x_lat, current_mission_.y_long, current_mission_.z_alt + current_home_.geo.altitude, 
                        map_point(0), map_point(1), map_point(2));

                    local_ecef = map_point - ecef_origin_;

                    goal_point = mavros::ftf::transform_frame_ecef_enu(local_ecef, map_origen_);

                    cmd.goal.position.x = goal_point(0);
                    cmd.goal.position.y = goal_point(1);
                    cmd.goal.position.z = goal_point(2);

                    cmd.has_goal = true;

                    cmd.state = planner_msgs::CommandState::PLANNING;

                    command_pub_.publish(cmd);

                    ROS_INFO("Goal set to: (%3f, %3f, %3f)", goal_point(0), goal_point(1), goal_point(2));
                } else
                {
                    ROS_WARN("Warning! Home Not Set!");
                }
            }
        } else if(((current_state_.mode == mavros_msgs::State::MODE_PX4_MANUAL) || ((current_state_.mode == mavros_msgs::State::MODE_PX4_POSITION)))
            && current_state_.armed)
        {
            ROS_INFO("RC control enabled.");

            cmd_state_ = CMD_STATE::RC;

            cmd.state = planner_msgs::CommandState::RC;
        }

        command_pub_.publish(cmd);
        return;
    }

    if(cmd_state_ == CMD_STATE::PLANNING)
    {
        cmd.state = planner_msgs::CommandState::PLANNING;
        cmd.has_goal = false;
        cmd.track_target = false;

        if(has_plan_ && current_state_.mode == mavros_msgs::State::MODE_PX4_LOITER)
        {
            ROS_INFO("Initializing trajectory tracking!");
            cmd_state_ = CMD_STATE::TRACK;

            cmd.state = planner_msgs::CommandState::TRACK;

            cmd.track_target = true;
        } else if((current_state_.mode == mavros_msgs::State::MODE_PX4_MANUAL) || ((current_state_.mode == mavros_msgs::State::MODE_PX4_POSITION))
            && current_state_.armed)
        {
            ROS_INFO("RC control enabled, abandoning planning.");
            cmd_state_ = CMD_STATE::RC;

            cmd.state = planner_msgs::CommandState::RC;
        }else if((current_state_.mode == mavros_msgs::State::MODE_PX4_RTL) || (current_state_.mode == mavros_msgs::State::MODE_PX4_LAND))
        {
            ROS_INFO("PX4 Autonomous mode enabled, abandoning planning.");
            cmd_state_ = CMD_STATE::IDLE;

            cmd.state = planner_msgs::CommandState::IDLE;
        }

        command_pub_.publish(cmd);
        return;
    }

    if(cmd_state_ == CMD_STATE::TRACK)
    {
        cmd.state = planner_msgs::CommandState::TRACK;
        cmd.has_goal = false;
        cmd.track_target = true;

        if(track_complete_)
        {
            mavros_msgs::SetMode plan_set_mode;
            plan_set_mode.request.custom_mode = "AUTO.LOITER";
            if(set_mode_client_.call(plan_set_mode) && plan_set_mode.response.mode_sent && has_goal_)
            {
                ROS_INFO("Completed Track! Return to Idle.");
                cmd_state_ = CMD_STATE::IDLE;

                cmd.state = planner_msgs::CommandState::IDLE;

                cmd.track_target = false;
            }
        }else if(((current_state_.mode == mavros_msgs::State::MODE_PX4_MANUAL) || ((current_state_.mode == mavros_msgs::State::MODE_PX4_POSITION)))
            && current_state_.armed)
        {
            ROS_INFO("RC control enabled, abandoning planning.");
            cmd_state_ = CMD_STATE::RC;

            cmd.state = planner_msgs::CommandState::RC;

            cmd.track_target = false;
        }else if((current_state_.mode == mavros_msgs::State::MODE_PX4_RTL) || (current_state_.mode == mavros_msgs::State::MODE_PX4_LAND))
        {
            ROS_INFO("PX4 Autonomous mode enabled, abandoning tracking.");
            cmd_state_ = CMD_STATE::IDLE;

            cmd.state = planner_msgs::CommandState::IDLE;

            cmd.track_target = false;
        }

        command_pub_.publish(cmd);
        return;
    }

    if(cmd_state_ == CMD_STATE::RC)
    {
        cmd.state = planner_msgs::CommandState::RC;
        cmd.has_goal = false;
        cmd.track_target = false;

        if(!(current_state_.mode == mavros_msgs::State::MODE_PX4_MANUAL) && !(current_state_.mode == mavros_msgs::State::MODE_PX4_POSITION))
        {
            ROS_INFO("Entering autonomous mode.");
            cmd_state_ = CMD_STATE::IDLE;

            cmd.state = planner_msgs::CommandState::IDLE;
        }

        command_pub_.publish(cmd);
        return;
    }
}