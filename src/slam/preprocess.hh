#pragma once
#include <pcl/filters/voxel_grid.h>
#include <memory>
#include "common/data_search.hh"

#include "livox_ros_driver/CustomMsg.h"
#include "sensors/imu.hh"
#include "sensors/point_types.hh"
#include "system/SystemManager.hh"

/**
 * @brief 传感器的处理和雷达特征提取以及去畸变
 */

namespace lio {

class FeatureExtractor;
class PointCloudProjector;
class LidarDistortionCorrector;

class PreProcessing {
   public:
    using DataSearcherIMU = DataSearcher<IMUData>;

   public:
    PreProcessing() = delete;
    explicit PreProcessing(SystemManager* sys);
    ~PreProcessing() = default;
    void Run();

   private:
    void InitFilter();
    PCLCloudXYZIRT::Ptr ConvertRosMessageToCloud(const sensor_msgs::PointCloud2Ptr& msg);
    // 计算每个点的时间偏移
    void ComputePointOffsetTime(const PCLCloudXYZIRT::Ptr& cloud, double lidar_rata = 10.0);
    std::pair<float, float> GetLidarPointMinMaxOffsetTime(const PCLCloudXYZIRT::Ptr& cloud);

   private:
    pcl::VoxelGrid<PCLPointXYZI> corner_voxel_filter_;
    pcl::VoxelGrid<PCLPointXYZI> plannar_voxel_filter_;
    PCLCloudXYZIRT::Ptr temp_cloud_ptr_;
    std::shared_ptr<FeatureExtractor> feature_extractor_ptr_;
    std::shared_ptr<PointCloudProjector> cloud_projector_ptr_;
    std::shared_ptr<LidarDistortionCorrector> lidar_distortion_corrector_ptr_;
    SystemManager* system_manager_;
};
}  // namespace lio