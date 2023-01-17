#include <planner/planner_algorithm.h>

#include <vector>
#include <math.h>
#include <Eigen/Dense>

// Dubins motion primitive types
enum class PRIMITIVE
{
    TURN_UP,
    TURN_DOWN,
    GO_STRAIGHT
};

// Domain boundary data -> rescaled to start / goal + domain_buffer
struct domain
{
    double xmin;
    double xmax;
    double ymin;
    double ymax;
};

// Data required for frontier set
struct motion_primitive
{
    Eigen::Vector3d pos;
    double cost;
    double starcost;

    int parent_idx;
    int idx;
    PRIMITIVE aci;
};

// Data required for visited set
struct visited_node
{
    Eigen::Vector3d pos;
    double cost;
    
    bool visited = false; // Default is unvisited
    int parent_idx;
    PRIMITIVE parent_aci;
};

// Here be bugs
inline int get_idx(std::vector<double>* grid_x, std::vector<double>* grid_y, std::vector<double>* grid_th, Eigen::Vector3d pos)
{
    int x_idx = 0;
    int y_idx = 0;
    int th_idx = 0;

    double xmin = std::abs(grid_x->at(0) - pos(0));
    double ymin = std::abs(grid_y->at(0) - pos(1));
    double thmin = std::abs(grid_th->at(0) - pos(2));

    for(int i = 1; i < grid_x->size(); i++)
    {
        if(std::abs(grid_x->at(i) - pos(0)) <= xmin)
        {
            xmin = std::abs(grid_x->at(i) - pos(0));
            x_idx = i;
        }
    }
    for(int i = 1; i < grid_y->size(); i++)
    {
        if(std::abs(grid_y->at(i) - pos(1)) <= ymin)
        {
            ymin = std::abs(grid_y->at(i) - pos(1));
            y_idx = i;
        }
    }
    for(int i = 1; i < grid_th->size(); i++)
    {
        if(std::abs(grid_th->at(i) - pos(2)) <= thmin)
        {
            thmin = std::abs(grid_th->at(i) - pos(2));
            th_idx = i;
        }
    }
    
    return th_idx*(grid_x->size())*(grid_y->size()) + y_idx*(grid_x->size()) + x_idx;
}

class hybrid_astar : public planner_algorithm
{
    private:
        domain domain_;

        double speed_;
        double turn_radius_;
        double step_length_;

        double dx_;
        double dy_;
        double dth_;

        Eigen::Vector3d start_pos_;
        Eigen::Vector3d goal_pos_;

        int start_idx_;
        int goal_idx_;

        std::vector<double> grid_x_;
        std::vector<double> grid_y_;
        std::vector<double> grid_th_;

        std::vector<visited_node> visited_;

    public:
        bool setup(Eigen::Vector3d start, Eigen::Vector3d goal, double domain_buffer);
        bool plan();

        hybrid_astar(double speed, double turn_radius, double step_length);
        virtual ~hybrid_astar();
};