#include <commander/commander.h>

commander::commander(const ros::NodeHandle &nh) : nh_(nh)
{
    state_sub_ = nh_.subscribe("mavros/state", 1, &commander::state_cb, this);
    waypoint_sub_ = nh_.subscribe("mavros/mission/waypoints", 1, &commander::waypoint_cb, this);
    home_sub_ = nh_.subscribe("mavros/home_position/home", 1, &commander::home_cb, this);
    pose_sub_ = nh_.subscribe("mavroslocal_position/pose", 1, &commander::pose_cb, this);

    set_commander_ = nh_.advertiseService("commander/set_commander", &commander::set_commander_cb, this);
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    set_controller_client_ = nh_.serviceClient<planner_msgs::SetController>("controller/set_controller");

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

    if(!home_set_)
    {
        home_set_ = true;
        ROS_INFO("Home Set!");
        ROS_INFO("Home Coordinates at: (%3f, %3f, %3f)", lat0, lon0, alt0);
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

void commander::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose_ = (*msg).pose;
}

bool commander::set_commander_cb(planner_msgs::SetCommander::Request& request, planner_msgs::SetCommander::Response& response)
{
    has_plan_ = request.has_plan;
    track_complete_ = request.track_complete;
    response.success = true;
    return true;
}

void commander::cmdloop_cb(const ros::TimerEvent &event)
{
    if(cmd_state_ == CMD_STATE::IDLE)
    {
        if((current_state_.mode == mavros_msgs::State::MODE_PX4_MISSION) && current_state_.armed)
        {
            mavros_msgs::SetMode plan_set_mode;
            plan_set_mode.request.custom_mode = "AUTO.LOITER";
            if(set_mode_client_.call(plan_set_mode) && plan_set_mode.response.mode_sent && has_goal_)
            {
                has_goal_ = false;
                if(home_set_)
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
                    // Set the goal yaw to the current yaw !! This will get more complicated as POI are introduced !!
                    planner_msgs::SetController set_controller;
                    set_controller.request.plan = true;
                    set_controller.request.track = false;

                    set_controller.request.goal.position.x = goal_point(0);
                    set_controller.request.goal.position.y = goal_point(1);
                    set_controller.request.goal.position.z = goal_point(2);
                    set_controller.request.goal.orientation = current_pose_.orientation;

                    set_controller.request.start = current_pose_;

                    if(set_controller_client_.call(set_controller) && set_controller.response.success)
                    {
                        cmd_state_ = CMD_STATE::PLANNING;
                        ROS_INFO("Start set to: (%3f, %3f, %3f)", current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
                        ROS_INFO("Goal set to: (%3f, %3f, %3f)", goal_point(0), goal_point(1), goal_point(2));
                        ROS_INFO("Initializing trajectory planning!");
                    }
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
        }
    }

    if(cmd_state_ == CMD_STATE::PLANNING)
    {
        if(has_plan_ && current_state_.mode == mavros_msgs::State::MODE_PX4_LOITER)
        {
            ROS_INFO("Initializing trajectory tracking!");
            has_plan_ = false;
            cmd_state_ = CMD_STATE::TRACK;
        } else if((current_state_.mode == mavros_msgs::State::MODE_PX4_MANUAL) || ((current_state_.mode == mavros_msgs::State::MODE_PX4_POSITION))
            && current_state_.armed)
        {
            ROS_INFO("RC control enabled, abandoning planning.");
            cmd_state_ = CMD_STATE::RC;
        } else if((current_state_.mode == mavros_msgs::State::MODE_PX4_RTL) || (current_state_.mode == mavros_msgs::State::MODE_PX4_LAND))
        {
            ROS_INFO("PX4 Autonomous mode enabled, abandoning planning.");
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
                ROS_INFO("Return to Idle.");
                cmd_state_ = CMD_STATE::IDLE;
            }
        } else if(((current_state_.mode == mavros_msgs::State::MODE_PX4_MANUAL) || ((current_state_.mode == mavros_msgs::State::MODE_PX4_POSITION)))
            && current_state_.armed)
        {
            cmd_state_ = CMD_STATE::RC;

            planner_msgs::SetController set_controller;
            set_controller.request.plan = false;
            set_controller.request.track = false;

            if(set_controller_client_.call(set_controller) && set_controller.response.success)
            {
                ROS_INFO("RC control enabled, abandoning tracking.");
            }    

        } else if((current_state_.mode == mavros_msgs::State::MODE_PX4_RTL) || (current_state_.mode == mavros_msgs::State::MODE_PX4_LAND))
        {
            cmd_state_ = CMD_STATE::IDLE;

            planner_msgs::SetController set_controller;
            set_controller.request.plan = false;
            set_controller.request.track = false;

            if(set_controller_client_.call(set_controller) && set_controller.response.success)
            {
                ROS_INFO("PX4 Autonomous mode enabled, abandoning tracking.");
            }  
        }
    }

    if(cmd_state_ == CMD_STATE::RC)
    {
        if(!(current_state_.mode == mavros_msgs::State::MODE_PX4_MANUAL) && !(current_state_.mode == mavros_msgs::State::MODE_PX4_POSITION))
        {
            ROS_INFO("Entering autonomous mode.");
            cmd_state_ = CMD_STATE::IDLE;
        }
    }
}