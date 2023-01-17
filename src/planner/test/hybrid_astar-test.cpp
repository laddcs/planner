#include <planner/hybrid_astar.h>

#include <gtest/gtest.h>
#include <iostream>

TEST(HybridAStarTest, TestSetup)
{
    // Create parameters to be passed to hybrid_astar object
    double speed = 3.0;
    double turn_radius = 8.0;
    double step_length = 3.0;

    // Start and Goal locations to be passed to setup
    Eigen::Vector3d start;
    Eigen::Vector3d goal;

    start << 25.0, 25.0, M_PI/4;
    goal << 75.0, 75.0, M_PI/4;

    double domain_buffer = 1.0;

    // Instanciate planner object
    hybrid_astar* HybridAStar = new hybrid_astar(speed, turn_radius, step_length);

    bool setup_success = HybridAStar->setup(start, goal, domain_buffer);

    ASSERT_TRUE(setup_success);
}