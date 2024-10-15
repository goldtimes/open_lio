#include "slam/localization.hh"
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
// #include <boost/filesystem/path.hpp>
#include <pcl/filters/crop_box.h>
#include <iostream>
#include "common/constant_variable.hh"
#include "common/lidar_utils.hh"
#include "system/ConfigParams.hh"
namespace lio {

Localization::Localization(SystemManager* sys) : sys_(sys) {
    // if (use_tile_map_) {
    //     tile_map_grid_size_ = ConfigParams::GetInstance().tile_map_grid_size_;
    //     tile_map_grid_size_half_ = tile_map_grid_size_ * 0.5;
    // }
}

void Localization::SetLidarPoseInformation() {
    double lidar_rot_std = ConfigParams::GetInstance().lidar_config.lidar_rotation_noise;
    double lidar_pos_std = ConfigParams::GetInstance().lidar_config.lidar_position_noise;
    lidar_pose_info.setZero();
    lidar_pose_info.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() / std::pow(lidar_rot_std, 2);
    lidar_pose_info.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() / std::pow(lidar_pos_std, 2);
}
void Localization::InitMatcher() {
    if (ConfigParams::GetInstance().front_config.registration_and_searcher_mode_ == kIcpOptimized) {
        matcher_ = std::make_shared<registration::ICPMatcher>(
            ConfigParams::GetInstance().front_config.registration_opti_iter_num_,
            ConfigParams::GetInstance().front_config.registration_local_map_size_,
            static_cast<float>(ConfigParams::GetInstance().front_config.registration_local_map_cloud_filter_size_),
            static_cast<float>(ConfigParams::GetInstance().front_config.registration_source_cloud_filter_size_),
            static_cast<double>(ConfigParams::GetInstance().front_config.registration_point_search_thres_),
            static_cast<double>(ConfigParams::GetInstance().front_config.registration_position_converge_thres_),
            static_cast<double>(ConfigParams::GetInstance().front_config.registration_rotation_converge_thres_),
            static_cast<double>(ConfigParams::GetInstance().front_config.registration_keyframe_delta_rotation_),
            static_cast<double>(ConfigParams::GetInstance().front_config.registration_keyframe_delta_dist_), true);
    }
}

void Localization::Run() {
    LOG(INFO) << "\033[1;32m----> Localization Started.\033[0m";
    // 加载地图
    {
        // 锁
        std::lock_guard<std::mutex> lck(mtx_global_map_);
        std::string global_map_path = kDataPath + kGlobalMapFileName;
        global_map_ptr_.reset(new PCLCloudXYZI);
        // 加载整个地图
        global_map_ptr_ = LoadGlobalMap(global_map_path);
        // 降采样
        if (global_map_ptr_) {
            const float filter_size = 0.3f;
            global_map_ptr_ = lidar_utils::VoxelGridCloud(global_map_ptr_, filter_size);
        }
    }
    // 是否使用tile_map
    if (use_tile_map_) {
        LoadTileMapIndices();
    }
    while (ros::ok()) {
        std::unique_lock<std::mutex> lck(sys_->mtx_cloud_cluster_queue_);
        while (sys_->cloud_cluster_queue_.empty()) {
            // 等待同步的数据
            if (!ros::ok()) {
                // 节点死掉了
                return;
            }
            // 如果没有同步好的数据，线程等待在这里
            sys_->cv_localization_.wait(lck);
        }
        // 有数据,取出最早的数据
        curr_cloud_cluster_ = sys_->cloud_cluster_queue_.front();
        sys_->cloud_cluster_queue_.pop_front();
        curr_time_us_ = curr_cloud_cluster_->timestamped_;
        if (!has_init_) {
            // 设置了初始化位姿
            has_init_ = Init();
            // 初始化预积分
            // TODO
            continue;
        }
        // TODO 状态传播然后匹配，根据位置更新地图
    }
}

bool Localization::Init() {
    // 根据初始化位姿加载地图
    Eigen::Matrix4d init_pose;
    {
        std::lock_guard<std::mutex> lck(mtx_init_pose_);
        if (!init_pose_optinal_.has_value()) {
            LOG_EVERY_N(INFO, 10) << "Please set initialization pose for localization by RViz -> 2D Pose Estimate";
            return false;
        }
        // 取出值并清空
        init_pose = init_pose_optinal_.value();
        init_pose_optinal_.reset();
    }
    // 根据init_pose来加载地图
    const auto local_map = LoadLocalMap(init_pose);
    if (local_map) {
        return false;
    }
    // 更新localmap
    UpdateLocalMapCloud(local_map);
    // 配准
    matcher_->AddCloudToLocalMap({*local_map});
    const bool match_result = matcher_->Match(curr_cloud_cluster_, init_pose);
    // 2米以内的参与计算
    const float fitness_score = matcher_->GetFitnessScore(2.0);
    if (match_result && fitness_score < 1.0) {
        // 这里没有创建一定范围匹配搜索
        LOG(INFO) << "get init pose, match success, match score:" << fitness_score;
    } else {
        LOG(WARNING) << "Initial registration failed. "
                     << "Please try to continue initialization and give a better initial pose.";
    }
    return match_result;
}

void Localization::UpdateLocalMapCloud(const PCLCloudXYZI::Ptr& new_cloud) {
    std::lock_guard<std::mutex> lck(mtx_local_map_);
    local_map_ptr_ = new_cloud;
    need_update_local_map_visualization_ = true;
}

PCLCloudXYZI::Ptr Localization::LoadLocalMap(const Eigen::Matrix4d& init_pose) {
    need_update_local_map_ = false;
    if (use_tile_map_) {
        // 计算地图的索引
        Eigen::Vector2d position = init_pose.block<2, 1>(0, 3);
        Eigen::Vector2i curr_grid_index;
        curr_grid_index[0] = std::floor((position[0] - tile_map_grid_size_half_) / tile_map_grid_size_);
        curr_grid_index[1] = std::floor((position[1] - tile_map_grid_size_half_) / tile_map_grid_size_);
        // 生成周边的地图索引
        const auto tile_map_indices = GenerateTileMapIndices(curr_grid_index, 1);
        // 根据索引加载地图
        for (const auto& tile_map_index : tile_map_indices) {
            // 索引不存在
            if (map_data_indices_.find(tile_map_index) == map_data_indices_.end()) {
                continue;
            }
            // 瓦片地图还没加载
            if (local_map_data_.find(tile_map_index) == local_map_data_.end()) {
                // 加载点云
                auto tile_map_cloud = LoadTileMap(tile_map_index);
                if (!tile_map_cloud) {
                    continue;
                }
                local_map_data_[tile_map_index] = lidar_utils::VoxelGridCloud(tile_map_cloud, 0.1);
                need_update_local_map_ = true;
            }
        }
        //  delete distant tile map cloud
        for (auto it = local_map_data_.begin(); it != local_map_data_.end();) {
            if ((it->first - curr_grid_index).norm() > 2) {
                local_map_data_.erase(it++);
                need_update_local_map_ = true;
            } else {
                ++it;
            }
        }

        PCLCloudXYZI::Ptr map_cloud(new PCLCloudXYZI);
        if (!need_update_local_map_) {
            return map_cloud;
        }

        map_cloud->clear();
        for (const auto pair : local_map_data_) {
            if (!pair.second) {
                continue;
            }
            *map_cloud += *(pair.second);
        }
        return map_cloud;
    } else {
        // 根据初始位置，加载一定范围内的点云
        PCLCloudXYZI::Ptr map_cloud(new PCLCloudXYZI);
        static bool first = true;
        static pcl::CropBox<PCLPointXYZI> crop_box_;

        // 第一次或者重新设置了初始化位姿
        if (first || !has_init_) {
            crop_box_.setInputCloud(global_map_ptr_);
            local_map_min_max_.clear();
            first = false;
        }

        if (local_map_min_max_.empty()) {
            need_update_local_map_ = true;
        } else {
            // 我们以pose为中心，创建了一个[-100~100]的地图
            for (int i = 0; i < 3; ++i) {
                // 如果离地图边缘还很远，我们就跳过
                if (std::fabs(init_pose(i, 3) - local_map_min_max_[i]) > 50.0 &&
                    std::fabs(init_pose(i, 3) - local_map_min_max_[i + 3]) > 50.0) {
                    continue;
                }
                // 否则我们需要更新地图的大小
                need_update_local_map_ = true;
                break;
            }
        }
        if (!need_update_local_map_) {
            return nullptr;
        }
        local_map_min_max_.resize(6);
        local_map_min_max_[0] = init_pose(0, 3) - 100.0;
        local_map_min_max_[1] = init_pose(1, 3) - 100.0;
        local_map_min_max_[2] = init_pose(2, 3) - 100.0;
        local_map_min_max_[3] = init_pose(0, 3) + 100.0;
        local_map_min_max_[4] = init_pose(1, 3) + 100.0;
        local_map_min_max_[5] = init_pose(2, 3) + 100.0;
        crop_box_.setMin(
            Eigen::Vector4d(local_map_min_max_[0], local_map_min_max_[1], local_map_min_max_[2], 1.0).cast<float>());
        crop_box_.setMax(
            Eigen::Vector4d(local_map_min_max_[3], local_map_min_max_[4], local_map_min_max_[5], 1.0).cast<float>());
        crop_box_.filter(*map_cloud);
        return map_cloud;
    }
}

PCLCloudXYZI::Ptr Localization::LoadTileMap(const Eigen::Vector2i& tile_map_index) {
    PCLCloudXYZI::Ptr cloud(new PCLCloudXYZI());
    std::string tile_map_path = kTileMapFolder + TILE_MAP_NAME(tile_map_index);
    int res = pcl::io::loadPCDFile(tile_map_path, *cloud);
    if (res == 0) {
        return cloud;
    }
    return nullptr;
}

void Localization::SetInitPose(const Eigen::Matrix4d& init_pose) {
    std::lock_guard<std::mutex> lck(mtx_init_pose_);
    init_pose_optinal_ = init_pose;
    has_init_ = false;
}

std::vector<Eigen::Vector2i> Localization::GenerateTileMapIndices(const Eigen::Vector2i& grid_index, int step_size) {
    // x: -1 ~ 1 y: -1 ~ 1
    std::vector<Eigen::Vector2i> neighbor_indices;
    for (int i = -step_size; i <= step_size; ++i) {
        for (int j = -step_size; j <= step_size; ++j) {
            Eigen::Vector2i step(i, j);
            neighbor_indices.emplace_back(grid_index + step);
        }
    }
}

PCLCloudXYZI::Ptr Localization::LoadGlobalMap(const std::string& global_map_path) {
    if (!boost::filesystem::exists(boost::filesystem::path(global_map_path))) {
        LOG(ERROR) << "global map path no exits";
        return nullptr;
    }
    PCLCloudXYZI::Ptr cloud(new PCLCloudXYZI());
    const auto result = pcl::io::loadPCDFile(global_map_path, *cloud);
    if (result == 0) {
        return cloud;
    }
    return nullptr;
}

void Localization::LoadTileMapIndices() {
    const std::string file_name = kTileMapFolder + kTileMapIndicesFileName;
    std::ifstream file(file_name);
    if (file.is_open()) {
        while (!file.eof()) {
            int x, y;
            file >> x >> y;
            map_data_indices_.emplace(x, y);
        }
    }
    file.close();
}

}  // namespace lio