#include <planner/hybrid_astar.h>

hybrid_astar::hybrid_astar(double speed, double turn_radius, double dt, double step_length) : planner_algorithm()
{
    speed_ = speed;
    turn_radius_ = turn_radius;
    step_length_ = step_length;

    dx_ = step_length_;
    dy_ = step_length_;
    dth_ = step_length_/turn_radius_;

    dt_ = dt;

    // Default domain size
    domain_.xmin = 0;
    domain_.xmax = 100;
    domain_.ymin = 0;
    domain_.ymax = 100; 
}

hybrid_astar::~hybrid_astar() {}

int hybrid_astar::get_idx(Eigen::Vector3d pos)
{
    if((pos(0) < domain_.xmin) || (pos(0) > domain_.xmax))
    {
        return -1;
    }
    if((pos(1) < domain_.ymin) || (pos(1) > domain_.ymax))
    {
        return -1;
    }
    // Make sure 0 <= yaw < 2*pi
    if(pos(2) < 0)
    {
        pos(2) = std::fmod(pos(2) + 2*M_PI, 2*M_PI);
    }

    const int x_idx = std::floor(LX_*(pos(0) - domain_.xmin)/(domain_.xmax - domain_.xmin));
    const int y_idx = std::floor(LY_*(pos(1) - domain_.ymin)/(domain_.ymax - domain_.ymin));
    const int th_idx = std::floor(LTH_*pos(2)/(2*M_PI));
    
    return th_idx*LX_*LY_ + y_idx*LX_ + x_idx;
}

double hybrid_astar::heuristic(Eigen::Vector3d pos)
{
    return std::sqrt(std::pow(pos(0) - goal_pos_(0), 2) + std::pow(pos(1) - goal_pos_(1), 2));
}

bool hybrid_astar::new_node(motion_primitive* node, motion_primitive* parent, PRIMITIVE aci)
{
    bool success = false;

    const double sx = parent->pos(0);
    const double sy = parent->pos(1);
    const double sth = parent->pos(2);

    double cx;
    double cy;
    double nth;

    Eigen::Vector3d pos;

    switch(aci)
    {
        case PRIMITIVE::TURN_UP:
            cx = sx - turn_radius_*std::sin(sth);
            cy = sy + turn_radius_*std::cos(sth);
            nth = std::fmod(sth + dth_, 2*M_PI);

            pos << cx + turn_radius_*std::sin(nth), cy - turn_radius_*std::cos(nth), nth;
            node->pos = pos;

            node->idx = get_idx(node->pos);
            
            if(node->idx == -1)
            {
                break;
            }

            node->parent_idx = parent->idx;
            node->aci = PRIMITIVE::TURN_UP;
            node->cost = parent->cost + step_length_;
            node->starcost = node->cost + heuristic(node->pos);

            success = true;
            break;

        case PRIMITIVE::TURN_DOWN:
            cx = sx + turn_radius_*std::sin(sth);
            cy = sy - turn_radius_*std::cos(sth);
            nth = std::fmod(sth - dth_ + 2*M_PI, 2*M_PI);

            pos << cx + turn_radius_*std::sin(-nth), cy + turn_radius_*std::cos(-nth), nth;
            node->pos = pos;

            node->idx = get_idx(node->pos);
            
            if(node->idx == -1)
            {
                break;
            }

            node->parent_idx = parent->idx;
            node->aci = PRIMITIVE::TURN_DOWN;
            node->cost = parent->cost + step_length_;
            node->starcost = node->cost + heuristic(node->pos);

            success = true;
            break;

        case PRIMITIVE::GO_STRAIGHT:
            pos << sx + step_length_*std::cos(sth), sy + step_length_*std::sin(sth), sth;
            node->pos = pos;

            node->idx = get_idx(node->pos);

            if(node->idx == -1)
            {
                break;
            }

            node->parent_idx = parent->idx;
            node->aci = PRIMITIVE::GO_STRAIGHT;
            node->cost = parent->cost + step_length_;
            node->starcost = node->cost + heuristic(node->pos);

            success = true;
            break;
    }
    return success;
}

bool hybrid_astar::setup(Eigen::Vector3d start, Eigen::Vector3d goal)
{
    start_pos_ = start;
    goal_pos_ = goal;

    // Scaling and translating the domain to match the area of interest
    double domain_factor = 9*step_length_;

    double xdist = (domain_.xmax + domain_.xmin)/2;
    double ydist = (domain_.ymax + domain_.ymin)/2;

    domain_.xmin = (start_pos_(0) + goal_pos_(0))/2 - xdist;
    domain_.xmax = (start_pos_(0) + goal_pos_(0))/2 + xdist;

    domain_.ymin = (start_pos_(1) + goal_pos_(1))/2 - ydist;
    domain_.ymax = (start_pos_(1) + goal_pos_(1))/2 + ydist;

    if((start_pos_(0) < domain_.xmin) || (goal_pos_(0) < domain_.xmin))
    {
        domain_.xmin -= std::max<double>(domain_.xmin - start_pos_(0), domain_.xmin - goal_pos_(0));
    }
    if((start_pos_(0) > domain_.xmax) || (goal_pos_(0) > domain_.xmax))
    {
        domain_.xmax += std::max<double>(start_pos_(0) - domain_.xmax, goal_pos_(0) - domain_.xmax);
    }
    if((start_pos_(1) < domain_.ymin) || (goal_pos_(1) < domain_.ymin))
    {
        domain_.ymin -= std::max<double>(domain_.ymin - start_pos_(1), domain_.ymin - goal_pos_(1));
    }
    if((start_pos_(1) > domain_.ymax) || (goal_pos_(1) > domain_.ymax))
    {
        domain_.ymax += std::max<double>(start_pos_(1) - domain_.ymax, goal_pos_(1) - domain_.ymax);
    }

    domain_.xmin -= domain_factor;
    domain_.xmax += domain_factor;

    domain_.ymin -= domain_factor;
    domain_.ymax += domain_factor;

    // Setting up the auxiliary grid
    LX_ = std::ceil((domain_.xmax - domain_.xmin)/dx_);
    LY_ = std::ceil((domain_.ymax - domain_.ymin)/dy_);
    LTH_ = std::ceil((2*M_PI)/dth_);

    start_idx_ = get_idx(start_pos_);
    goal_idx_ = get_idx(goal_pos_);

    if((start_idx_ == -1) || (goal_idx_ == -1))
    {
        return false;
    }

    motion_primitive start_node;
    start_node.aci = PRIMITIVE::START;
    start_node.pos = start_pos_;
    start_node.cost = 0.0;
    start_node.starcost = heuristic(start_node.pos);
    start_node.idx = start_idx_;
    start_node.parent_idx = -1;

    frontier_.push(start_node);

    visited_.resize(LX_*LY_*LTH_);

    return true;
}

bool hybrid_astar::plan()
{
    int c_idx;
    motion_primitive node;
    
    while(!frontier_.empty())
    {
        current_ = frontier_.top();
        c_idx = current_.idx;
        frontier_.pop();

        visited_[c_idx].handle = NOHANDLE;
        visited_[c_idx].cost = current_.cost;
        visited_[c_idx].parent_aci = current_.aci;
        visited_[c_idx].parent_idx = current_.parent_idx;
        visited_[c_idx].pos = current_.pos;
        visited_[c_idx].visited = true;

        if(c_idx == goal_idx_)
        {
            return true;
        }

        for(PRIMITIVE aci = PRIMITIVE::TURN_UP; aci < PRIMITIVE::START; aci = PRIMITIVE(aci + 1))
        {
            if(new_node(&node, &current_, aci))
            {
                if(!visited_[node.idx].visited)
                {
                    if(visited_[node.idx].handle == NOHANDLE)
                    {
                        visited_[node.idx].handle = frontier_.push(node);
                    } else
                    {
                        double current_starcost = (*visited_[node.idx].handle).starcost;
                        if(node.starcost < current_starcost)
                        {
                            frontier_.decrease(visited_[node.idx].handle, node);
                        }
                    }
                }
            }
        }
    }

    return false;
}

std::vector<visited_node> hybrid_astar::get_path()
{
    std::vector<visited_node> path;
    visited_node p_node = visited_[goal_idx_];
    
    while(true)
    {
        path.emplace(path.begin(), p_node);
        if(p_node.parent_idx == -1)
        {
            break;
        }
        p_node = visited_[p_node.parent_idx];
    }

    return path;
}

std::vector<std::array<double, 7>> hybrid_astar::get_trajectory()
{
    const int disc = 3; // Need tp set as a parameter
    int t_idx = 0;
    double dt = dt_/disc;

    double sx;
    double sy;
    double sth;

    double cx;
    double cy;
    double nth;

    PRIMITIVE current_aci;

    std::vector<visited_node> path = get_path();
    std::vector<std::array<double, 7>> trajectory;
    trajectory.resize((path.size()-1)*4);

    for(int i = 0; i < path.size()-1; i++)
    {
        sx = path[i].pos(0);
        sy = path[i].pos(1);
        sth = path[i].pos(2);
        current_aci = path[i+1].parent_aci;

        switch(current_aci)
        {
            case PRIMITIVE::TURN_UP:
                cx = sx - turn_radius_*std::sin(sth);
                cy = sy + turn_radius_*std::cos(sth);

                for(int j = 0; j <= disc; j++)
                {
                    nth = std::fmod(sth + j*dt*dth_, 2*M_PI);
                    trajectory[t_idx][0] = cx + turn_radius_*std::sin(nth);
                    trajectory[t_idx][1] = cy - turn_radius_*std::cos(nth);
                    trajectory[t_idx][2] = nth;

                    trajectory[t_idx][3] = turn_radius_*dth_*std::cos(nth);
                    trajectory[t_idx][4] = turn_radius_*dth_*std::sin(nth);
                    trajectory[t_idx][5] = 0.0;

                    trajectory[t_idx][6] = dt*t_idx;

                    t_idx ++;
                }

                break;
            case PRIMITIVE::TURN_DOWN:
                cx = sx + turn_radius_*std::sin(sth);
                cy = sy - turn_radius_*std::cos(sth);

                for(int j = 0; j <= disc; j++)
                {
                    nth = std::fmod(sth - j*dt*dth_, 2*M_PI);
                    trajectory[t_idx][0] = cx + turn_radius_*std::sin(-nth);
                    trajectory[t_idx][1] = cy + turn_radius_*std::cos(-nth);
                    trajectory[t_idx][2] = nth;

                    trajectory[t_idx][3] = turn_radius_*dth_*std::cos(-nth);
                    trajectory[t_idx][4] = -turn_radius_*dth_*std::sin(-nth);
                    trajectory[t_idx][5] = 0.0;

                    trajectory[t_idx][6] = dt*t_idx;

                    t_idx ++;
                }

                break;
            case PRIMITIVE::GO_STRAIGHT:
                for(int j = 0; j <= disc; j++)
                {
                    trajectory[t_idx][0] = sx + j*dt*step_length_*std::cos(sth);
                    trajectory[t_idx][1] = sy + j*dt*step_length_*std::sin(sth);
                    trajectory[t_idx][2] = sth;

                    trajectory[t_idx][3] = step_length_*std::cos(sth);
                    trajectory[t_idx][4] = step_length_*std::sin(sth);
                    trajectory[t_idx][5] = 0.0;

                    trajectory[t_idx][6] = dt*t_idx;
                    t_idx ++;
                }

                break;
        }
    }
    
    return trajectory;
}

bool hybrid_astar::cleanup()
{
    // Default domain size
    domain_.xmin = 0;
    domain_.xmax = 100;
    domain_.ymin = 0;
    domain_.ymax = 100; 

    visited_.clear();
    frontier_.clear();

    return true;
}

// Getter methods for testing
double hybrid_astar::get_dom_xmin() {return domain_.xmin;}
double hybrid_astar::get_dom_xmax() {return domain_.xmax;}
double hybrid_astar::get_dom_ymin() {return domain_.ymin;}
double hybrid_astar::get_dom_ymax() {return domain_.ymax;}

int hybrid_astar::get_start_idx() {return start_idx_;}
int hybrid_astar::get_goal_idx() {return goal_idx_;}
