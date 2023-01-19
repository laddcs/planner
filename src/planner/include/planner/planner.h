#include <ros/ros.h>

#include <string.h>

#include <planner/hybrid_astar.h>
#include <planner_msgs/PlanPath.h>

#include <geometry_msgs/Pose.h>

class planner
{
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::ServiceServer planner_;

        std::string planner_type_;

        std::shared_ptr<planner_algorithm> planner_algorithm_;

        // Hybrid-A* Parameters
        double speed_;
        double R_;
        double L_;
        double dt_;

        geometry_msgs::Pose start_;
        geometry_msgs::Pose goal_;

        bool planner_cb(planner_msgs::PlanPath::Request& request, planner_msgs::PlanPath::Response& response);

    public:
        planner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
};