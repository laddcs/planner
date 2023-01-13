#include <ros/ros.h>

#include <string>

#include <planner/common.h>

#include <geometry_msgs/Pose.h>

#include <mavros/frame_tf.h>

#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/CommandCode.h>

#include <planner_msgs/CommandState.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/Constants.hpp>

enum class CMD_STATE
{
    IDLE,
    PLANNING,
    TRACK,
    RC
};

class commander
{
    private:
        ros::NodeHandle nh_;

        ros::Subscriber state_sub_;
        ros::Subscriber waypoint_sub_;
        ros::Subscriber home_sub_;

        ros::Publisher command_pub_;
        ros::ServiceClient set_mode_client_;
        ros::Timer cmdloop_timer_;

        mavros_msgs::HomePosition current_home_;
        geometry_msgs::Pose goal_pose_;
        mavros_msgs::State current_state_;
        mavros_msgs::Waypoint current_mission_;

        bool home_set_;
        bool has_goal_;
        bool has_plan_;
        bool track_complete_;
        CMD_STATE cmd_state_;

        Eigen::Vector3d ecef_origin_;
        Eigen::Vector3d map_origen_;
        GeographicLib::Geocentric earth;

        void state_cb(const mavros_msgs::State::ConstPtr& msg);
        void waypoint_cb(const mavros_msgs::WaypointList::ConstPtr& msg);
        void home_cb(const mavros_msgs::HomePosition::ConstPtr& msg);
        void cmdloop_cb(const ros::TimerEvent &event);

    public:
        commander(const ros::NodeHandle &nh);
};