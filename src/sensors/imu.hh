#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
namespace lio {
class IMU {
   public:
    IMU() = default;
    IMU(double time, const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc)
        : timestamp_(time), acc_(acc), gyro_(gyro) {
    }
    double timestamp_;
    Eigen::Vector3d acc_;
    Eigen::Vector3d gyro_;
};
}  // namespace lio

using IMUPtr = std::shared_ptr<lio::IMU>;