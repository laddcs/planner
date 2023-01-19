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
    int x_idx = 0;
    int y_idx = 0;
    int th_idx = 0;

    double xmin = std::abs(grid_x_[0] - pos(0));
    double ymin = std::abs(grid_y_[0] - pos(1));
    double thmin = std::abs(grid_th_[0] - pos(2));

    if((pos(0) < grid_x_[0]) || (pos(0) > grid_x_.back()))
    {
        return -1;
    }

    if((pos(1) < grid_y_[0]) || (pos(1) > grid_y_.back()))
    {
        return -1;
    }

    for(int i = 1; i < grid_x_.size(); i++)
    {
        if(std::abs(grid_x_[i] - pos(0)) <= xmin)
        {
            xmin = std::abs(grid_x_[i] - pos(0));
            x_idx = i;
        }
    }
    for(int i = 1; i < grid_y_.size(); i++)
    {
        if(std::abs(grid_y_[i] - pos(1)) <= ymin)
        {
            ymin = std::abs(grid_y_[i] - pos(1));
            y_idx = i;
        }
    }
    for(int i = 1; i < grid_th_.size(); i++)
    {
        if(std::abs(grid_th_[i] - pos(2)) <= thmin)
        {
            thmin = std::abs(grid_th_[i] - pos(2));
            th_idx = i;
        }
    }
    
    return th_idx*(grid_x_.size())*(grid_y_.size()) + y_idx*(grid_x_.size()) + x_idx;
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
            nth = std::fmod(sth - dth_, 2*M_PI);

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

bool hybrid_astar::setup(Eigen::Vector3d start, Eigen::Vector3d goal, double domain_buffer)
{
    start_pos_ = start;
    goal_pos_ = goal;

    // Scaling and translating the domain to match the area of interest
    // Need to fix the scaling -> doesnt pass think test
    double xbuff = (domain_.xmax - domain_.xmin)*domain_buffer;
    double ybuff = (domain_.ymax - domain_.ymin)*domain_buffer;

    double xdist = (domain_.xmax + domain_.xmin)/2;
    double ydist = (domain_.ymax + domain_.ymin)/2;

    domain_.xmin = (start_pos_(0) + goal_pos_(0))/2 - xdist - xbuff;
    domain_.xmax = (start_pos_(0) + goal_pos_(0))/2 + xdist + xbuff;

    domain_.ymin = (start_pos_(1) + goal_pos_(1))/2 - ydist - ybuff;
    domain_.ymax = (start_pos_(1) + goal_pos_(1))/2 + ydist + ybuff;

    // Setting up the auxiliary grid
    int LX = std::ceil((domain_.xmax - domain_.xmin)/dx_);
    int LY = std::ceil((domain_.ymax - domain_.ymin)/dy_);
    int LTH = std::ceil((2*M_PI)/dth_);

    for(int i = 0; i < LX; i++)
    {
        grid_x_.emplace_back<double>(domain_.xmin + ((double)(dx_*i)));
    }
    for(int i = 0; i < LY; i++)
    {
        grid_y_.emplace_back<double>(domain_.ymin + ((double)(dy_*i)));
    }
    for(int i = 0; i < LTH; i++)
    {
        grid_th_.emplace_back<double>((double)(dth_*i));
    }

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

    visited_.resize(LX*LY*LTH);

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

        if(visited_[c_idx].visited)
        {
            continue;
        }

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
                frontier_.push(node);
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

std::vector<Eigen::Vector4d> hybrid_astar::get_trajectory()
{
    int disc = 3;
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
    std::vector<Eigen::Vector4d> trajectory;
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
                    trajectory[t_idx](0) = cx + turn_radius_*std::sin(nth);
                    trajectory[t_idx](1) = cy - turn_radius_*std::cos(nth);
                    trajectory[t_idx](2) = nth;
                    trajectory[t_idx](3) = dt*t_idx;
                    t_idx ++;
                }

                break;
            case PRIMITIVE::TURN_DOWN:
                cx = sx + turn_radius_*std::sin(sth);
                cy = sy - turn_radius_*std::cos(sth);

                for(int j = 0; j <= disc; j++)
                {
                    nth = std::fmod(sth - j*dt*dth_, 2*M_PI);
                    trajectory[t_idx](0) = cx + turn_radius_*std::sin(-nth);
                    trajectory[t_idx](1) = cy + turn_radius_*std::cos(-nth);
                    trajectory[t_idx](2) = nth;
                    trajectory[t_idx](3) = dt*t_idx;
                    t_idx ++;
                }

                break;
                break;
            case PRIMITIVE::GO_STRAIGHT:
                for(int j = 0; j <= disc; j++)
                {
                    trajectory[t_idx](0) = sx + j*dt*step_length_*std::cos(sth);
                    trajectory[t_idx](1) = sy + j*dt*step_length_*std::sin(sth);
                    trajectory[t_idx](2) = sth;
                    trajectory[t_idx](3) = dt*t_idx;
                    t_idx ++;
                }
                break;
        }
    }
    
    return trajectory;
}

// Getter methods for testing
double hybrid_astar::get_dom_xmin() {return domain_.xmin;}
double hybrid_astar::get_dom_xmax() {return domain_.xmax;}
double hybrid_astar::get_dom_ymin() {return domain_.ymin;}
double hybrid_astar::get_dom_ymax() {return domain_.ymax;}

int hybrid_astar::get_start_idx() {return start_idx_;}
int hybrid_astar::get_goal_idx() {return goal_idx_;}
