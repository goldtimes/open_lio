#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
#include "sensors/data.hh"

namespace lio {
struct NaviStateData : DataBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void SetPose(const Eigen::Matrix4d& pose_4d) {
        P_ = pose_4d.block<3, 1>(0, 3);
        R_ = pose_4d.block<3, 3>(0, 0);
    }

    [[nodiscard]] Eigen::Matrix4d Pose() const {
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose.block<3, 1>(0, 3) = P_;
        pose.block<3, 3>(0, 0) = R_;
        return Pose();
    }

    Eigen::Vector3d P_;
    Eigen::Vector3d V_;
    Eigen::Matrix3d R_;
    Eigen::Vector3d ba_;
    Eigen::Vector3d bg_;
    Eigen::Matrix<double, 15, 15> info;
};
}  // namespace lio

using NaviStatePtr = std::shared_ptr<lio::NaviStateData>;
