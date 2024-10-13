#pragma once
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include "common/compare_function.hh"
#include "registration/icp_matcher.hh"
#include "slam/splict_map.hh"
#include "system/SystemManager.hh"

namespace lio {
class Localization {
   public:
    explicit Localization(SystemManager* sys) : sys_(sys) {
    }
    void Run();

    void SetInitPose(const Eigen::Matrix4d& init_pose);

   private:
    PCLCloudXYZI::Ptr LoadGlobalMap(const std::string& global_map_path);
    PCLCloudXYZI::Ptr LoadLocalMap(const Eigen::Matrix4d& init_pose);
    void LoadTileMapIndices();
    std::vector<Eigen::Vector2i> GenerateTileMapIndices(const Eigen::Vector2i& grid_index, int step_size);
    PCLCloudXYZI::Ptr LoadTileMap(const Eigen::Vector2i& tile_map_index);
    bool Init();

   private:
    SystemManager* sys_ = nullptr;

    std::mutex mtx_global_map_;
    std::mutex mtx_local_map_;
    std::mutex mtx_lidar_cloud_;
    PCLCloudXYZI::Ptr global_map_ptr_;
    PCLCloudXYZI::Ptr local_map_ptr_;
    PCLCloudXYZI::Ptr lidar_cloud_ptr_;
    // 子地图的大小
    std::vector<double> local_map_min_max_;
    bool need_update_local_map_;
    // 瓦片地图的大小
    double tile_map_grid_size_;
    double tile_map_grid_size_half_;
    // 初始位姿
    std::optional<Eigen::Matrix4d> init_pose_optinal_;
    std::mutex mtx_init_pose_;

    PointCloudClusterPtr curr_cloud_cluster_;
    uint64_t curr_time_us_;
    bool has_init_;
    // 瓦片地图的时候
    bool use_tile_map_;
    // 存放点云地图以及对应的索引
    std::map<Eigen::Vector2i, PCLCloudXYZI::Ptr, LessVec2i> local_map_data_;
    std::set<Eigen::Vector2i, LessVec2i> map_data_indices_;
};
}  // namespace lio