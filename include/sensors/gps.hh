#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
#include "sensors/data.hh"
namespace lio {
struct GPSData : DataBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GPSData() = default;
    GPSData(uint64_t us, double longti, double lati, double alti)
        : DataBase(us), longtitude(longti), latitude(lati), altitude(alti) {
    }
    double longtitude;
    double latitude;
    double altitude;
    Eigen::Vector3d local_xyz;
    Eigen::Vector3d local_xyz_velocity;
    Eigen::Matrix3d local_orientation;
};
}  // namespace lio

using GPSPtr = std::shared_ptr<lio::GPSData>;