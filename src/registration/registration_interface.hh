#pragma once
#include "common/data_type.hh"
#include "lidar_process/pointcloud_cluster.hh"

namespace registration {
class RegistrationInterface {
   public:
    // 配准
    virtual bool Match(const lio::PointCloudClusterPtr& cloud_cluster, Eigen::Matrix4d& init_pose) = 0;
    virtual ~RegistrationInterface() = default;
    virtual void AddCloudToLocalMap(std::initializer_list<PCLCloudXYZI> cloud_list) = 0;
    // 返回值不可忽略且不可以修改，获取配准分数
    [[nodiscard]] virtual float GetFitnessScore(float max_range) const = 0;
};
}  // namespace registration