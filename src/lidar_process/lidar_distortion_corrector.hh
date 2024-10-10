#pragma once

#include <glog/logging.h>
#include "common/data_search.hh"
#include "common/data_type.hh"
#include "common/interpolator.hh"
#include "sensors/imu.hh"

namespace lio {
class LidarDistortionCorrector {
   public:
    explicit LidarDistortionCorrector(const Eigen::Matrix4d& T_IL);
    void SetDataSearcher(std::shared_ptr<DataSearcher<IMUData>> imu_data_searcher);
    bool SetRefTime(uint64_t ref_time);
    bool UnidstortPoint(const Eigen::Vector3f& orig_point, Eigen::Vector3f& out_point, float relative_time);

   private:
    uint64_t ref_time;
    // 插值之后的imu数据
    Eigen::Vector3d inter_pos;
    Eigen::Quaterniond ref_quat_inv_;
    // lidar->imu外参信息
    Eigen::Matrix4d T_IL_ = Eigen::Matrix4d::Identity();
    std::shared_ptr<DataSearcher<IMUData>> imu_data_searcher_ = nullptr;
};
}  // namespace lio