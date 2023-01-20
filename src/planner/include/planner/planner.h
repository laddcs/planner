#include <ros/ros.h>

#include <string.h>

#include <planner/hybrid_astar.h>
#include <planner_msgs/PlanPath.h>

#include <std_msgs/Time.h>

#include <geometry_msgs/Pose.h>

inline double extract_yaw(const geometry_msgs::Quaternion q)
{
    return atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
}

inline geometry_msgs::Quaternion euler2Quat(const double roll, const double pitch, const double yaw)
{
    geometry_msgs::Quaternion quat;

    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);

    quat.w = cr * cp * cy + sr * sp * sy;
    quat.x = sr * cp * cy - cr * sp * sy;
    quat.y = cr * sp * cy + sr * cp * sy;
    quat.z = cr * cp * sy - sr * sp * cy;

    return quat;
}


class planner
{
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::ServiceServer planner_;

        std::string planner_type_;

        std::shared_ptr<hybrid_astar> planner_algorithm_;

        // Hybrid-A* Parameters
        double speed_;
        double R_;
        double L_;
        double dt_;

        geometry_msgs::Pose start_;
        geometry_msgs::Pose goal_;

        bool planner_cb(planner_msgs::PlanPath::Request& request, planner_msgs::PlanPath::Response& response);

    public:
        planner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
};