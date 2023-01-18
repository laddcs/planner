#include <planner/planner_algorithm.h>

#include <iostream>
#include <vector>
#include <math.h>
#include <boost/heap/fibonacci_heap.hpp>
#include <Eigen/Dense>

// Dubins motion primitive types
enum PRIMITIVE
{
    TURN_UP,
    TURN_DOWN,
    GO_STRAIGHT,
    START
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

struct compare_primitive
{
    bool operator()(const motion_primitive& node1, const motion_primitive& node2) const
    {
        return node1.starcost > node2.starcost;
    }
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
        boost::heap::fibonacci_heap<motion_primitive, boost::heap::compare<compare_primitive>> frontier_;

        motion_primitive current_;

        int get_idx(Eigen::Vector3d pos);
        double heuristic(Eigen::Vector3d pos);
        bool new_node(motion_primitive* node, motion_primitive* parent, PRIMITIVE aci);

    public:
        bool setup(Eigen::Vector3d start, Eigen::Vector3d goal, double domain_buffer);
        bool plan();
        std::vector<visited_node> get_path();

        hybrid_astar(double speed, double turn_radius, double step_length);
        virtual ~hybrid_astar();

        // Getter methods for testing
        double get_dom_xmin();
        double get_dom_xmax();
        double get_dom_ymin();
        double get_dom_ymax();

        int get_start_idx();
        int get_goal_idx();
};