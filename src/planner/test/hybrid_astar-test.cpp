#include <planner/hybrid_astar.h>

#include <gtest/gtest.h>
#include <iostream>

TEST(HybridAStarTest, TestSetup)
{
    // Create parameters to be passed to hybrid_astar object
    double speed = 3.0;
    double turn_radius = 8.0;
    double step_length = 3.0;
    double dt = 1.0;

    // Start and Goal locations to be passed to setup
    Eigen::Vector3d start;
    Eigen::Vector3d goal;

    start << 25.0, 25.0, 0;
    goal << 75.0, 75.0, 0;

    // Instanciate planner object
    hybrid_astar* HybridAStar = new hybrid_astar(speed, turn_radius, dt, step_length);

    bool setup_success = HybridAStar->setup(start, goal);

    ASSERT_DOUBLE_EQ(HybridAStar->get_dom_xmin(), 0.0);
    ASSERT_DOUBLE_EQ(HybridAStar->get_dom_xmax(), 100.0);
    ASSERT_DOUBLE_EQ(HybridAStar->get_dom_ymin(), 0.0);
    ASSERT_DOUBLE_EQ(HybridAStar->get_dom_ymax(), 100.0);

    ASSERT_EQ(HybridAStar->get_LX(), 34);
    ASSERT_EQ(HybridAStar->get_LY(), 34);
    ASSERT_EQ(HybridAStar->get_LTH(), 17);

    ASSERT_EQ(HybridAStar->get_start_idx(), 315);
    ASSERT_EQ(HybridAStar->get_goal_idx(), 910);

    ASSERT_TRUE(setup_success);
}

TEST(HybridAStarTest, TestPlan)
{
    // Create parameters to be passed to hybrid_astar object
    double speed = 3.0;
    double turn_radius = 8.0;
    double step_length = 3.0;
    double dt = 1.0;

    // Start and Goal locations to be passed to setup
    Eigen::Vector3d start;
    Eigen::Vector3d goal;

    start << 0.0, 0.0, 0;
    goal << 75, 75, M_PI;

    // Instanciate planner object
    hybrid_astar* HybridAStar = new hybrid_astar(speed, turn_radius, dt, step_length);

    bool setup_success = HybridAStar->setup(start, goal);
    bool plan_success = HybridAStar->plan();

    ASSERT_TRUE(setup_success);
    ASSERT_TRUE(plan_success);

    // Test path construction
    std::cout << "\nResulting Path:\n";

    std::vector<visited_node> path = HybridAStar->get_path();
    for(int i = 0; i < path.size(); i++)
    {
        std::cout << "(" << path[i].pos(0) << ", " << path[i].pos(1) << ", " << path[i].pos(2) << ")\n";
    }

    std::cout << "\n";

    // Test trajectory construction
    std::vector<std::array<double, 7>> trajectory = HybridAStar->get_trajectory();

    std::cout << "\nResulting Trajectory:\n";

    for(int i = 0; i < trajectory.size(); i++)
    {
        std::cout << "(" << trajectory[i][0] << ", "  << trajectory[i][1] << ", " << trajectory[i][2] << ")" 
            << " (" << trajectory[i][3] << ", "  << trajectory[i][4] << ", " << trajectory[i][5] << ")"
            << " TS: " << trajectory[i][6] << "\n";
    }

    std::cout << "\n";
}