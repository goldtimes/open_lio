#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
#include "sensors/data.hh"
namespace lio {
struct IMUData : DataBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    IMUData() = default;
    IMUData(uint64_t us, const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc)
        : DataBase(us), acc_(acc), gyro_(gyro) {
    }
    Eigen::Vector3d acc_;
    Eigen::Vector3d gyro_;
    // 有些imu不只输出加速度和角速度信息，还输出朝向信息
    Eigen::Quaterniond orientation_;
};
}  // namespace lio

using IMUPtr = std::shared_ptr<lio::IMUData>;