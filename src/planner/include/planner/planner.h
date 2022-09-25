#include <boost/heap/fibonacci_heap.hpp>
#include <boost/intrusive/list.hpp>

using namespace boost::heap;
using namespace boost::intrusive;

// Enumerating possible actions
enum ACTION
{
    TURN_UP,
    TURN_DOWN,
    NO_TURN,
    NO_ACTION
};

// Pose of a node
struct position
{
    float x;
    float y;
    float theta;
};

// Data needed when a node is in the frontier
struct frontier_node
{
    float cost;
    float starcost;
    float load;
    int idx;
    int parent;
    position pos;
    ACTION aci;
};

class planner
{
    private:
        fibonacci_heap<frontier_node> frontier;

        // Grid Cells
        float* grid_x;
        float* grid_y;
        float* grid_theta;

    public:
        planner();
};