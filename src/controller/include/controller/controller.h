#include <ros/ros.h>

#include <vector>

#include <controller/common.h>

#include <planner_msgs/SetController.h>
#include <planner_msgs/SetCommander.h>
#include <planner_msgs/PlanPath.h>

#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

class controller
{
    private:
        ros::NodeHandle nh_;

        ros::Subscriber setpoint_sub_;
        ros::Subscriber state_sub_;
        ros::Subscriber pose_sub_;

        ros::Publisher setpoint_pub_;

        ros::Timer controller_timer_;

        ros::ServiceClient set_commander_client_;
        ros::ServiceClient set_mode_client_;
        ros::ServiceServer set_controller_;
        ros::ServiceClient plan_path_client_;

        ros::Time last_request;
        ros::Time track_start_;

        bool plan_;
        bool track_;
        bool tracking_;
        int track_idx_;
        std::vector<planner_msgs::PathSetpoint> trajectory_;

        geometry_msgs::Pose goal_;
        geometry_msgs::Pose start_;
        geometry_msgs::PoseStamped current_px4_setpoint_;
        geometry_msgs::PoseStamped current_offboard_setpoint_;
        geometry_msgs::Pose current_pose_;
        mavros_msgs::State current_state_;

        void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void setpoint_cb(const mavros_msgs::PositionTarget::ConstPtr& msg);
        void state_cb(const mavros_msgs::State::ConstPtr& msg);
        void controller_cb(const ros::TimerEvent &event);
        bool set_controller_cb(planner_msgs::SetController::Request& request, planner_msgs::SetController::Response& response);
        
    public:
        controller(const ros::NodeHandle &nh);
};