#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
#include "sensors/data.hh"
namespace lio {
struct IMUData : DataBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    IMUData() = default;
    IMUData(double time, const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc)
        : DataBase(time), acc_(acc), gyro_(gyro) {
    }
    Eigen::Vector3d acc_;
    Eigen::Vector3d gyro_;
    Eigen::Quaterniond orientation_;
};
}  // namespace lio

using IMUPtr = std::shared_ptr<lio::IMUData>;