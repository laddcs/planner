#include <ros/ros.h>

#include <planner_msgs/PlanPath.h>

#include <geometry_msgs/Pose.h>

class planner
{
    private:
        ros::NodeHandle nh_;

        ros::ServiceServer planner_;

        geometry_msgs::Pose start_;
        geometry_msgs::Pose goal_;

        bool planner_cb(planner_msgs::PlanPath::Request& request, planner_msgs::PlanPath::Response& response);

    public:
        planner(const ros::NodeHandle &nh);
};