#include <planner/planner.h>

planner::planner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : nh_(nh), nh_private_(nh_private)
{
    planner_ = nh_.advertiseService("planner/plan_path", &planner::planner_cb, this);

    nh_private_.param<std::string>("planner_type", planner_type_, "hybrid-astar");

    // Parameters for Hybrid-A* search
    nh_private_.param<double>("speed", speed_, 3.0);
    nh_private_.param<double>("dt", dt_, 1.0);
    nh_private_.param<double>("turn_radius", R_, 8.0);

    L_ = speed_ * dt_;

    if(planner_type_ == "hybrid-astar")
    {
        planner_algorithm_ = std::make_shared<hybrid_astar>(speed_, R_, dt_, L_);
    }
}

bool planner::planner_cb(planner_msgs::PlanPath::Request& request, planner_msgs::PlanPath::Response& response)
{
    start_ = request.start;
    goal_ = request.goal;

    Eigen::Vector3d start;
    Eigen::Vector3d goal;
    double height = request.goal.position.z;

    std::vector<Eigen::Vector4d> trajectory;
    std::vector<planner_msgs::PathSetpoint> final_trajectory;

    start << start_.position.x, start_.position.y, extract_yaw(start_.orientation);
    goal << goal_.position.x, goal_.position.y, extract_yaw(goal_.orientation);

    if(planner_algorithm_->setup(start, goal) && planner_algorithm_->plan())
    {
        trajectory = planner_algorithm_->get_trajectory();
        final_trajectory.resize(trajectory.size());
        response.success = true;
    } else 
    {
        response.success = false;
        return true;
    }

    for(int i = 0; i < final_trajectory.size(); i++)
    {
        final_trajectory[i].pos.position.x = trajectory[i](0);
        final_trajectory[i].pos.position.y = trajectory[i](1);
        final_trajectory[i].pos.position.z = height;

        final_trajectory[i].pos.orientation = euler2Quat(0, 0, trajectory[i](2));

        final_trajectory[i].target_time.data = ros::Duration(trajectory[i](3));
    }

    response.path.plan = final_trajectory;

    return true;
}