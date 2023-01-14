#include <ros/ros.h>

#include <planner/common.h>
#include <planner_msgs/CommandState.h>

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>

class controller
{
    private:
        ros::NodeHandle nh_;

        ros::Subscriber cmd_sub_;

        ros::Publisher setpoint_pub_;

        ros::ServiceClient set_mode_client_;

        bool track_;
        Eigen::Vector3d goal_;
        planner_msgs::CommandState current_cmd_;

        void cmd_msg_cb(const planner_msgs::CommandState::ConstPtr& msg);
        
    public:
        controller(const ros::NodeHandle &nh);
};