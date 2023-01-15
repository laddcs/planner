#include <ros/ros.h>

#include <planner/common.h>
#include <planner_msgs/SetController.h>
#include <planner_msgs/SetCommander.h>

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

        ros::Publisher setpoint_pub_;

        ros::Timer controller_timer_;

        ros::ServiceClient set_commander_client_;
        ros::ServiceClient set_mode_client_;
        ros::ServiceServer set_controller_;

        ros::Time last_request;

        bool plan_;
        bool track_;

        geometry_msgs::Pose goal_;
        geometry_msgs::Pose start_;
        geometry_msgs::PoseStamped current_px4_setpoint_;
        geometry_msgs::PoseStamped current_offboard_setpoint_;
        mavros_msgs::State current_state_;

        void setpoint_cb(const mavros_msgs::PositionTarget::ConstPtr& msg);
        void state_cb(const mavros_msgs::State::ConstPtr& msg);
        void controller_cb(const ros::TimerEvent &event);
        bool set_controller_cb(planner_msgs::SetController::Request& request, planner_msgs::SetController::Response& response);
        
    public:
        controller(const ros::NodeHandle &nh);
};