#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#include <Eigen/Dense>

inline Eigen::Vector3d toEigen(const geometry_msgs::Point &p) {
  Eigen::Vector3d ev3(p.x, p.y, p.z);
  return ev3;
}

inline Eigen::Vector3d toEigen(const geometry_msgs::Vector3 &v3) {
  Eigen::Vector3d ev3(v3.x, v3.y, v3.z);
  return ev3;
}

inline Eigen::Vector4d toEigen(const geometry_msgs::Quaternion& msg)
{
    Eigen::Vector4d ev4(msg.w, msg.x, msg.y, msg.z);
    return ev4;
}

inline Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4d &q) {
    Eigen::Matrix3d rotmat;
    rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3), 2 * q(1) * q(2) - 2 * q(0) * q(3),
        2 * q(0) * q(2) + 2 * q(1) * q(3),

        2 * q(0) * q(3) + 2 * q(1) * q(2), q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
        2 * q(2) * q(3) - 2 * q(0) * q(1),

        2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3),
        q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
    return rotmat;
}