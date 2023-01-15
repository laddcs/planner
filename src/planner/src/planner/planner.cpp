#include <planner/planner.h>

planner::planner(const ros::NodeHandle &nh) : nh_(nh)
{
    planner_ = nh_.advertiseService("planner/plan_path", &planner::planner_cb, this);
}

bool planner::planner_cb(planner_msgs::PlanPath::Request& request, planner_msgs::PlanPath::Response& response)
{
    start_ = request.start;
    goal_ = request.goal;

    return true;
}