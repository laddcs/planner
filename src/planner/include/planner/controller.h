#include <ros/ros.h>

#include <planner/common.h>
#include <planner_msgs/CommandState.h>

#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

class controller
{
    private:
        ros::NodeHandle nh_;

        ros::Subscriber cmd_sub_;
        ros::Subscriber setpoint_sub_;

        ros::Publisher setpoint_pub_;

        ros::Timer controller_timer_;

        ros::ServiceClient set_mode_client_;

        ros::Time last_request;

        Eigen::Vector3d goal_;
        planner_msgs::CommandState current_cmd_;
        geometry_msgs::PoseStamped current_px4_setpoint_;
        geometry_msgs::PoseStamped current_offboard_setpoint_;

        void cmd_msg_cb(const planner_msgs::CommandState::ConstPtr& msg);
        void setpoint_cb(const mavros_msgs::PositionTarget::ConstPtr& msg);
        void controller_cb(const ros::TimerEvent &event);
        
    public:
        controller(const ros::NodeHandle &nh);
};