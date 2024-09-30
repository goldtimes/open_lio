#pragma once

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace lio {
// ros 格式到eigen的格式变换

inline uint64_t RosTimeToUs(const std_msgs::Header_<std::allocator<void>>& header) {
    return header.stamp.sec * 1e6 + header.stamp.nsec / 1e3;
}

inline Eigen::Vector3d RosV3dToEigen(const geometry_msgs::Vector3_<std::allocator<void>>& vec) {
    return Eigen::Vector3d{vec.x, vec.y, vec.z};
}

inline Eigen::Vector3d RosPointToEigen(const geometry_msgs::Point_<std::allocator<void>>& point) {
    return Eigen::Vector3d{point.x, point.y, point.z};
}

inline Eigen::Quaterniond RosQuaternionToEigen(const geometry_msgs::Quaternion_<std::allocator<void>>& q) {
    return Eigen::Quaterniond{q.w, q.x, q.y, q.z};
}

}  // namespace lio
