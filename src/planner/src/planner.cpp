#include <planner/planner.h>

planner::planner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : nh_(nh), nh_private_(nh_private)
{
    planner_ = nh_.advertiseService("planner/plan_path", &planner::planner_cb, this);

    nh_private_.param<std::string>("planner_type", planner_type_, "hybrid-astar");

    // Parameters for Hybrid-A* search
    nh_private_.param<double>("speed", speed_, 3.0);
    nh_private_.param<double>("turn_radius", R_, 8.0);
    nh_private_.param<double>("step_length", L_, 3.0);

    if(planner_type_ == "hybrid-astar")
    {
        planner_algorithm_ = std::make_shared<hybrid_astar>(speed_, R_, L_);
    }
}

bool planner::planner_cb(planner_msgs::PlanPath::Request& request, planner_msgs::PlanPath::Response& response)
{
    start_ = request.start;
    goal_ = request.goal;

    return true;
}