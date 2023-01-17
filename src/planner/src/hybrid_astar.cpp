#include <planner/hybrid_astar.h>

hybrid_astar::hybrid_astar(double speed, double turn_radius, double step_length) : planner_algorithm()
{
    speed_ = speed;
    turn_radius_ = turn_radius;
    step_length_ = step_length;

    dx_ = step_length_;
    dy_ = step_length_;
    dth_ = step_length_/turn_radius_;

    // Default domain size
    domain_.xmin = 0;
    domain_.xmax = 100;
    domain_.ymin = 0;
    domain_.ymax = 100;
}

hybrid_astar::~hybrid_astar() {}

bool hybrid_astar::setup(Eigen::Vector3d start, Eigen::Vector3d goal, double domain_buffer)
{
    start_pos_ = start;
    goal_pos_ = goal;

    // Scaling and translating the domain to match the area of interest
    // Need to fix the scaling -> doesnt pass think test
    double xbuff = (domain_.xmax - domain_.xmin)*domain_buffer;
    double ybuff = (domain_.ymax - domain_.ymin)*domain_buffer;

    domain_.xmin += (start_pos_(0) + goal_pos_(0))/2 - (domain_.xmin + domain_.xmax)/2 - xbuff;
    domain_.xmax += (start_pos_(0) + goal_pos_(0))/2 - (domain_.xmin + domain_.xmax)/2 + xbuff;

    domain_.ymin += (start_pos_(1) + goal_pos_(1))/2 - (domain_.ymin + domain_.ymax)/2 - ybuff;
    domain_.ymax += (start_pos_(1) + goal_pos_(1))/2 - (domain_.ymin + domain_.ymax)/2 + ybuff;

    // Setting up the auxiliary grid
    int LX = std::ceil(xbuff/dx_);
    int LY = std::ceil(ybuff/dy_);
    int LTH = std::ceil((2*M_PI)/dth_);

    for(int i = 0; i < LX; i++)
    {
        grid_x_.emplace_back<double>(domain_.xmin + (dx_*i));
    }
    for(int i = 0; i < LY; i++)
    {
        grid_x_.emplace_back<double>(domain_.ymin + (dy_*i));
    }
    for(int i = 0; i < LTH; i++)
    {
        grid_x_.emplace_back<double>(dth_*i);
    }

    // Set the size of the visited set
    visited_.resize(LX*LY*LTH);

    return true;
}