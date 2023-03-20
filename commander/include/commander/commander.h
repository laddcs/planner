#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <thread>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>

#include <mavros/frame_tf.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/CompanionProcessStatus.h>

#include <planner_msgs/SetController.h>
#include <planner_msgs/SetCommander.h>
#include <planner_msgs/PlanPath.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/Constants.hpp>

#include "commander/common.h"

class commander
{
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle control_nh_;

        ros::Subscriber state_sub_;
        ros::Subscriber setpoint_sub_;
        ros::Subscriber waypoint_sub_;
        ros::Subscriber home_sub_;
        ros::Subscriber pose_sub_;

        ros::Publisher setpoint_pub_;
        ros::Publisher status_pub_;

        ros::ServiceClient plan_path_client_;
        ros::ServiceClient set_mode_client_;
        ros::ServiceClient waypoint_pull_client_;
        
        ros::Timer cmdloop_timer_;
        ros::Timer ctlloop_timer_;

        mavros_msgs::HomePosition current_home_;
        mavros_msgs::State current_state_;

        // Planning Info
        mavros_msgs::Waypoint current_mission_;
        geometry_msgs::Pose goal_pose_;
        geometry_msgs::Pose current_pose_;

        // Tracking Info
        ros::Time last_offboard_request_;
        ros::Time last_waypoint_request_;
        ros::Time last_hold_request_;
        ros::Time track_start_;

        int track_idx_;
        geometry_msgs::PoseStamped current_px4_setpoint_;
        geometry_msgs::PoseStamped current_offboard_setpoint_;
        std::vector<planner_msgs::PathSetpoint> trajectory_;

        // State Flags
        bool guidance_;
        bool home_set_;
        bool has_goal_;
        bool transfer_set_; // Flag for setting setpoint for offboard transition
        bool planning_; // Internal to controller process
        bool has_plan_;
        bool tracking_; // Internal to controller process
        bool track_complete_;
        CMD_STATE cmd_state_;
        mavros_msgs::CompanionProcessStatus system_status_;

        // Coordinate Transform
        Eigen::Vector3d ecef_origin_;
        Eigen::Vector3d map_origen_;
        GeographicLib::Geocentric earth;

        // Callabacks
        void state_cb(const mavros_msgs::State::ConstPtr& msg);
        void setpoint_cb(const mavros_msgs::PositionTarget::ConstPtr& msg);
        void waypoint_cb(const mavros_msgs::WaypointList::ConstPtr& msg);
        void home_cb(const mavros_msgs::HomePosition::ConstPtr& msg);
        void cmdloop_cb(const ros::TimerEvent &event);
        void ctlloop_cb(const ros::TimerEvent &event);
        void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

    public:
        commander(const ros::NodeHandle &nh, const ros::NodeHandle &control_nh);
};