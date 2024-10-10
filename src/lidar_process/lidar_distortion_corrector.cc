#include "lidar_process/lidar_distortion_corrector.hh"

namespace lio {
LidarDistortionCorrector::LidarDistortionCorrector(const Eigen::Matrix4d& T_IL) : T_IL_(T_IL) {
}

void LidarDistortionCorrector::SetDataSearcher(std::shared_ptr<DataSearcher<IMUData>> imu_data_searcher) {
    imu_data_searcher_ = imu_data_searcher;
}

bool LidarDistortionCorrector::SetRefTime(uint64_t curr_cloud_time) {
    ref_time = curr_cloud_time;  // 帧首或者帧尾的时间
    // 插值获得curr_cloud_time点的姿态
    IMUData imu_data_l, imu_data_r;
    if (!imu_data_searcher_->SearchNearestTwoData(ref_time, imu_data_l, imu_data_r)) {
        LOG(WARNING) << "Don't have valid imu data to set reference";
        return false;
    }
    // 插值
    ref_quat_inv_ =
        Interpolator::InterpolateQuaternionSlerp(imu_data_l.orientation_, imu_data_r.orientation_,
                                                 imu_data_l.timestamped_, imu_data_r.timestamped_, curr_cloud_time)
            .inverse();
    return true;
}

// 这里去畸变，但是没有转到lidar坐标系中
bool LidarDistortionCorrector::UnidstortPoint(const Eigen::Vector3f& orig_point, Eigen::Vector3f& out_point,
                                              float relative_time) {
    Eigen::Vector3d pt_lidar = orig_point.cast<double>();
    Eigen::Quaterniond q_curr;
    const auto curr_t =
        static_cast<uint64_t>(static_cast<int64_t>(ref_time) + static_cast<int64_t>(relative_time * 1.0e6));
    IMUData imu_data_r, imu_data_l;
    if (!imu_data_searcher_->SearchNearestTwoData(curr_t, imu_data_l, imu_data_r)) {
        LOG(ERROR) << "Search current point orientation error. Normally this error should not occur. "
                      "Please check the timestamp of the lidar point!";
        return false;
    } else {
        q_curr = Interpolator::InterpolateQuaternionSlerp(imu_data_l.orientation_, imu_data_r.orientation_,
                                                          imu_data_l.timestamped_, imu_data_r.timestamped_, curr_t);
    }
    Eigen::Vector3d pt_imu = T_IL_.block<3, 3>(0, 0) * pt_lidar + T_IL_.block<3, 1>(0, 3);
    //  T_LI * T_IW_ref * T_WI_current * T _IL * P_lidar
    Eigen::Vector3d pt_corrected = ref_quat_inv_ * q_curr * pt_imu;
    out_point = pt_corrected.cast<float>();
    return true;
}
}  // namespace lio