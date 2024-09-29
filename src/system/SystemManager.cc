#include "system/SystemManager.hh"
#include <glog/logging.h>
#include "system/ConfigParams.hh"

namespace lio {
SystemManager::SystemManager(std::shared_ptr<ros::NodeHandle> nh) {
    nh_ = std::move(nh);
    InitConfigParams();
    InitLidarModel();
    InitRosPublishers();
    InitRosSubscribers();
}

SystemManager::~SystemManager() {
}

void SystemManager::InitConfigParams() {
    // 获得单例对象
    ConfigParams& config = ConfigParams::GetInstance();
    nh_->param<std::string>("sensor_topic/lidar_topic", config.lidar_topic_, "lidar");
    LOG(INFO) << "lidar_topic:" << config.lidar_topic_;
    nh_->param<std::string>("sensor_topic/imu_topic", config.imu_topic_, "imu");
    LOG(INFO) << "imu_topic:" << config.imu_topic_;
    nh_->param<bool>("has_encoder", config.has_encoder_, false);
    if (!config.has_encoder_) {
        nh_->param<std::string>("sensor_topic/encoder_topic", config.encoder_topic_, "odom");
        LOG(INFO) << "encoder_topic:" << config.encoder_topic_;
    }
    int slam_mode = 0;
    nh_->param<int>("slam_mode", slam_mode, 0);
    switch (slam_mode) {
        case 0:
            slam_mode_ = SLAM_MODE::UNKNOW;
            LOG(ERROR) << "unknow slam mode";
            break;
        case 1:
            slam_mode_ = SLAM_MODE::MAPPING;
            LOG(INFO) << "[SLAM MODE]: mapping";
            break;
        case 2:
            slam_mode_ = SLAM_MODE::LOCALIZATION;
            LOG(INFO) << "[SLAM MODE]: LOCALIZATION";
            break;
    }
    // lidar config
    nh_->param<std::string>("lidar/lidar_type", config.lidar_config.lidar_type, "");
    LOG(INFO) << "lidar/lidar_type:" << config.lidar_config.lidar_type;
    nh_->param<int>("lidar/lidar_scan_num", config.lidar_config.lidar_scan_num, 16);
    LOG(INFO) << "lidar/lidar_scan_num:" << config.lidar_config.lidar_scan_num;
    nh_->param<int>("lidar/lidar_horizon_scan_num", config.lidar_config.lidar_horizon_scan_num, 1024);
    LOG(INFO) << "lidar/lidar_horizon_scan_num:" << config.lidar_config.lidar_horizon_scan_num;
    nh_->param<double>("lidar/v_res", config.lidar_config.v_res, 0.2);
    LOG(INFO) << "lidar/v_res:" << config.lidar_config.v_res;
    nh_->param<double>("lidar/lidar_lower_angle", config.lidar_config.lidar_lower_angle, 15.0);
    LOG(INFO) << "lidar/lidar_lower_angle:" << config.lidar_config.lidar_lower_angle;
    nh_->param<double>("lidar/lidar_point_time_scale", config.lidar_config.lidar_point_time_scale, 0.0);
    LOG(INFO) << "lidar/lidar_point_time_scale:" << config.lidar_config.lidar_point_time_scale;
    nh_->param<int>("lidar/lidar_jump_span", config.lidar_config.lidar_jump_span);
    LOG(INFO) << "lidar/lidar_jump_span:" << config.lidar_config.lidar_jump_span;
    nh_->param<double>("lidar/lidar_position_noise", config.lidar_config.lidar_position_noise, 0.0001);
    nh_->param<double>("lidar/lidar_rotation_noise", config.lidar_config.lidar_rotation_noise, 0.0001);
    LOG(INFO) << "lidar/lidar_position_noise:" << config.lidar_config.lidar_position_noise;
    LOG(INFO) << "lidar/lidar_rotation_noise:" << config.lidar_config.lidar_rotation_noise;
    nh_->param<double>("lidar/lidar_min_dist", config.lidar_config.lidar_min_dist, 0.2);
    nh_->param<double>("lidar/lidar_max_dist", config.lidar_config.lidar_max_dist, 200.0);
    LOG(INFO) << "lidar/lidar_min_dist:" << config.lidar_config.lidar_min_dist;
    LOG(INFO) << "lidar/lidar_max_dist:" << config.lidar_config.lidar_max_dist;
    // imu config
    nh_->param<double>("imu/init_acc_bias", config.imu_config.init_acc_bias, 0.0);
    nh_->param<double>("imu/init_gyro_bias", config.imu_config.init_gyro_bias, 0.0);
    LOG(INFO) << "imu/init_acc_bias:" << config.imu_config.init_acc_bias;
    LOG(INFO) << "imu/init_gyro_bias:" << config.imu_config.init_gyro_bias;
    nh_->param<double>("imu/imu_acc_noise_std", config.imu_config.imu_acc_noise_std, 0.0);
    nh_->param<double>("imu/imu_gryo_noise_std", config.imu_config.imu_gryo_noise_std, 0.0);
    LOG(INFO) << "imu/imu_acc_noise_std:" << config.imu_config.imu_acc_noise_std;
    LOG(INFO) << "imu/imu_gryo_noise_std:" << config.imu_config.imu_gryo_noise_std;
    nh_->param<double>("imu/imu_acc_rw_noise", config.imu_config.imu_acc_rw_noise, 0.0);
    nh_->param<double>("imu/imu_gyro_rw_noise", config.imu_config.imu_gyro_rw_noise, 0.0);
    LOG(INFO) << "imu/imu_gyro_rw_noise:" << config.imu_config.imu_gyro_rw_noise;
    LOG(INFO) << "imu/imu_acc_rw_noise:" << config.imu_config.imu_acc_rw_noise;
    // encoder config
    nh_->param<double>("encoder/encoder_position_noise", config.encoder_config.encoder_position_noise, 0.0001);
    nh_->param<double>("encoder/encoder_rotation_noise", config.encoder_config.encoder_rotation_noise, 0.05);
    LOG(INFO) << "encoder/encoder_position_noise:" << config.encoder_config.encoder_position_noise;
    LOG(INFO) << "encoder/encoder_rotation_noise:" << config.encoder_config.encoder_rotation_noise;
}

void SystemManager::InitLidarModel() {
}
void SystemManager::InitRosPublishers() {
}

void SystemManager::InitRosSubscribers() {
}
}  // namespace lio