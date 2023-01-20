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

    start << 0.0, 0.0, M_PI/4;
    goal << 0.0, 150.0, M_PI/4;

    // Instanciate planner object
    hybrid_astar* HybridAStar = new hybrid_astar(speed, turn_radius, dt, step_length);

    bool setup_success = HybridAStar->setup(start, goal);

    ASSERT_DOUBLE_EQ(HybridAStar->get_dom_xmin(), -50.0);
    ASSERT_DOUBLE_EQ(HybridAStar->get_dom_xmax(), 50.0);
    ASSERT_DOUBLE_EQ(HybridAStar->get_dom_ymin(), 0.0);
    ASSERT_DOUBLE_EQ(HybridAStar->get_dom_ymax(), 150.0);

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
    goal << -110.0, 15.0, 0;

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
    std::vector<Eigen::Vector4d> trajectory = HybridAStar->get_trajectory();

    std::cout << "\nResulting Trajectory:\n";

    for(int i = 0; i < trajectory.size(); i++)
    {
        std::cout << trajectory[i](0) << ", "  << trajectory[i](1) 
            << ", " << trajectory[i](2) << " TS: " << trajectory[i](3) << "\n";
    }

    std::cout << "\n";
}