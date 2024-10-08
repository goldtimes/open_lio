#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <string>
/**
 * @brief 单例模式的参数配置类
 */
namespace lio {

using V3d = Eigen::Vector3d;
using M4d = Eigen::Matrix4d;

struct IMUConfig {
    double init_acc_bias{};
    double init_gyro_bias{};
    double imu_acc_noise_std{};
    double imu_gryo_noise_std{};
    double imu_acc_rw_noise{};
    double imu_gyro_rw_noise{};
    bool has_orientation{};
};

struct LidarConfig {
    std::string lidar_type;
    int lidar_scan_num;          // 多少线的雷达
    int lidar_horizon_scan_num;  // 每行的点数
    double v_res;                // 垂直分辨率
    double lidar_lower_angle;    // 垂直方向的起始角度
    double lidar_point_time_scale;
    int lidar_jump_span;          // 跳点
    double lidar_position_noise;  // 平移噪声
    double lidar_rotation_noise;  // 旋转噪声
    double lidar_min_dist;        // 最小距离
    double lidar_max_dist;        // 最大距离
};
struct EncoderConfig {
    double encoder_position_noise;
    double encoder_rotation_noise;
};

struct LoopClosureConfig {};

struct SystemConfig {
    double keyframe_angle_thres{};
    double keyframe_distance_thres{};
    bool enable_loopclosure{};
    bool enable_vis_global_map{};
    // 显示全局地图的时候降采样
    double global_vis_res{};
};

// 不同lidar_odom模式下的配准参数
struct FrontConfig {
    // loam feature parameters
    std::string registration_and_searcher_mode_{};
    float loam_feature_corner_thres_{};
    float loam_feature_planar_thres_{};
    float loam_feature_planar_voxel_filter_size_{};
    float loam_feature_corner_voxel_filter_size_{};

    // registration parameters
    float registration_local_corner_filter_size_{};
    float registration_local_planar_filter_size_{};
    float registration_local_map_cloud_filter_size_{};
    float registration_source_cloud_filter_size_{};
    float registration_line_ratio_thres_{};
    float registration_point_search_thres_{};
    float registration_point_to_planar_thres_{};
    float registration_position_converge_thres_{};
    float registration_rotation_converge_thres_{};
    double registration_keyframe_delta_dist_{};
    double registration_keyframe_delta_rotation_{};
    double registration_ndt_voxel_size_{};
    double registration_ndt_outlier_threshold_{};
    int registration_ndt_min_points_in_voxel_{};
    int registration_ndt_max_points_in_voxel_{};
    int registration_ndt_min_effective_pts_{};
    int registration_ndt_capacity_{};
    int registration_local_map_size_{};
    int registration_local_planar_map_size_{};
    int registration_local_corner_map_size_{};
    int registration_opti_iter_num_{};
    // frontend parameters
    int frontend_fusion_opti_iters_{};  // optimization iterations
};

class ConfigParams {
   public:
    ConfigParams(ConfigParams&& other) = delete;
    ConfigParams(const ConfigParams& other) = delete;
    ConfigParams operator=(const ConfigParams& other) = delete;
    static ConfigParams& GetInstance();

   private:
    ConfigParams() = default;
    static std::unique_ptr<ConfigParams> instance_;

   public:
    // imu 参数
    IMUConfig imu_config;
    // lidar 参数
    LidarConfig lidar_config;
    // encoder 参数
    EncoderConfig encoder_config;
    // 系统配置参数
    SystemConfig system_config;
    // 前端配准的参数
    FrontConfig front_config;
    // 回环检测参数
    LoopClosureConfig lp_config;

    int imu_data_buffer_size_;

    V3d gravity_vec{0, 0, -9.81};
    double gravity_norm{9.81};

    // 点云地图的切割大小
    double split_map_size_;
    // eskf or ieskf
    std::string ekf_model_;
    // 紧耦合还是松耦合的方式
    std::string fusion_method_;
    // 前端的迭代次数
    int frontend_iter_;
    // lidar->encoder的外参
    Eigen::Matrix4d T_EL;
    // lidar-to-imu外参
    Eigen::Matrix4d T_IL_;
    // imu
    std::string imu_topic_;
    // lidar
    std::string lidar_topic_;
    // 轮速计
    bool has_encoder_;
    std::string encoder_topic_;
};
}  // namespace lio