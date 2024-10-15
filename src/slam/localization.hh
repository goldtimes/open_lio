#pragma once
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include "common/compare_function.hh"
#include "registration/icp_matcher.hh"
#include "registration/registration_interface.hh"
#include "slam/split_map.hh"
#include "system/SystemManager.hh"

namespace lio {
class PreIntegration;

class Localization {
   public:
    explicit Localization(SystemManager* sys);
    void Run();

    void SetInitPose(const Eigen::Matrix4d& init_pose);

   private:
    PCLCloudXYZI::Ptr LoadGlobalMap(const std::string& global_map_path);
    PCLCloudXYZI::Ptr LoadLocalMap(const Eigen::Matrix4d& init_pose);
    void LoadTileMapIndices();
    std::vector<Eigen::Vector2i> GenerateTileMapIndices(const Eigen::Vector2i& grid_index, int step_size);
    PCLCloudXYZI::Ptr LoadTileMap(const Eigen::Vector2i& tile_map_index);
    bool Init();
    void UpdateLocalMapCloud(const PCLCloudXYZI::Ptr& new_cloud);

    void SetLidarPoseInformation();
    void InitMatcher();

   private:
    SystemManager* sys_ = nullptr;

    std::shared_ptr<registration::RegistrationInterface> matcher_ = nullptr;
    std::shared_ptr<PreIntegration> imu_pre_integration_;

    Eigen::Matrix<double, 6, 6> lidar_pose_info;

    std::mutex mtx_global_map_;
    std::mutex mtx_local_map_;
    std::mutex mtx_lidar_cloud_;
    PCLCloudXYZI::Ptr global_map_ptr_;
    PCLCloudXYZI::Ptr local_map_ptr_;
    PCLCloudXYZI::Ptr lidar_cloud_ptr_;
    // 子地图的大小
    std::vector<double> local_map_min_max_;
    bool need_update_local_map_;

    std::atomic_bool has_init_{false};
    std::atomic_bool need_update_local_map_visualization_{false};
    // 瓦片地图的大小
    double tile_map_grid_size_;
    double tile_map_grid_size_half_;
    // 初始位姿
    std::optional<Eigen::Matrix4d> init_pose_optinal_;
    std::mutex mtx_init_pose_;

    PointCloudClusterPtr curr_cloud_cluster_;
    uint64_t curr_time_us_;

    std::atomic_bool has_init_{false};
    std::atomic_bool need_update_local_map_visualization_{false};
    // 瓦片地图的时候, 并不推荐
    bool use_tile_map_;
    // 存放点云地图以及对应的索引
    std::map<Eigen::Vector2i, PCLCloudXYZI::Ptr, LessVec2i> local_map_data_;
    std::set<Eigen::Vector2i, LessVec2i> map_data_indices_;
};
}  // namespace lio