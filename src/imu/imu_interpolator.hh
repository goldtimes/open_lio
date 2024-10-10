#pragma once
#include "common/interpolator.hh"
#include "sensors/imu.hh"

namespace imu {
inline lio::IMUData IMUInterpolator(const lio::IMUData& imu_l, lio::IMUData& imu_r, double t_us) {
    lio::IMUData imu_data;
    imu_data.acc_ =
        lio::Interpolator::InterpolateVectorLerp(imu_l.acc_, imu_r.acc_, imu_l.timestamped_, imu_r.timestamped_, t_us);
    imu_data.gyro_ = lio::Interpolator::InterpolateVectorLerp(imu_l.gyro_, imu_r.gyro_, imu_l.timestamped_,
                                                              imu_r.timestamped_, t_us);
    imu_data.orientation_ = lio::Interpolator::InterpolateQuaternionSlerp(imu_l.orientation_, imu_r.orientation_,
                                                                          imu_l.timestamped_, imu_r.timestamped_, t_us);

    return imu_data;
}
}  // namespace imu