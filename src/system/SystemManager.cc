#include "system/SystemManager.hh"
#include <glog/logging.h>
#include "lidar_process/lidar_model.hh"
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

void SystemManager::InitServer() {
    map_saver_server_ = nh_->advertiseService("/save_map", &SystemManager::SaveMap, this);
}

void SystemManager::InitLidarModel() {
    LidarModel::Instance(ConfigParams::GetInstance().lidar_config.lidar_type);
}

void SystemManager::InitRosPublishers() {
    // path
    lidar_path_pub = nh_->advertise<nav_msgs::Path>("lidar_path", 5);
    imu_path_pub = nh_->advertise<nav_msgs::Path>("imu_path", 5);
    encoder_path_pub = nh_->advertise<nav_msgs::Path>("encoder_path", 5);
    // odom
    lidar_odom_pub = nh_->advertise<nav_msgs::Odometry>("lidar_odom", 5);
    imu_odom_pub = nh_->advertise<nav_msgs::Odometry>("imu_odom", 5);
    encoder_odom_pub = nh_->advertise<nav_msgs::Odometry>("encoder_odom", 5);
    // cloud
    curr_scan_pub = nh_->advertise<sensor_msgs::PointCloud2>("curr_scan", 5);
    local_map_pub = nh_->advertise<sensor_msgs::PointCloud2>("local_map", 5);
    global_map_pub = nh_->advertise<sensor_msgs::PointCloud2>("global_map", 5);
    corner_cloud_pub = nh_->advertise<sensor_msgs::PointCloud2>("corner_cloud", 5);
    plannar_cloud_pub = nh_->advertise<sensor_msgs::PointCloud2>("plannar_cloud", 5);
    // keyframe
    keyframe_path_pub = nh_->advertise<sensor_msgs::PointCloud2>("keyframe_path", 5);
    keyframe_path_cloud_pub = nh_->advertise<sensor_msgs::PointCloud2>("keyframe_cloud", 5);
    curr_keyframe_pub = nh_->advertise<sensor_msgs::PointCloud2>("curr_keyframe_cloud", 5);
}

void SystemManager::InitRosSubscribers() {
    if (LidarModel::Instance()->lidar_type == LidarType::AVIA) {
        lidar_sub =
            nh_->subscribe(ConfigParams::GetInstance().lidar_topic_, 100, &SystemManager::LidarLivoxCallback, this);
    } else {
        lidar_sub = nh_->subscribe(ConfigParams::GetInstance().lidar_topic_, 100,
                                   &SystemManager::LidarStandarMsgCallback, this);
    }
    imu_sub = nh_->subscribe(ConfigParams::GetInstance().imu_topic_, 1000, &SystemManager::IMUCallback, this);
    if (ConfigParams::GetInstance().has_encoder_) {
        odom_sub =
            nh_->subscribe(ConfigParams::GetInstance().encoder_topic_, 1000, &SystemManager::EncoderCallback, this);
    }
    if (slam_mode_ == SLAM_MODE::LOCALIZATION) {
        init_pose_sub = nh_->subscribe("/init_pose", 1, &SystemManager::InitposeCallback, this);
    }
}

void SystemManager::LidarLivoxCallback(const livox_ros_driver::CustomMsg::ConstPtr& livox_msg) {
    // livox的类型要特殊处理,将custmsg 转换为sensor_msgs;
    using PointField = sensor_msgs::PointField;

    static sensor_msgs::PointCloud2 cloud_ros;
    static bool init_ros_cloud_type = false;
    // 初始化一次ros_cloud的头
    if (!init_ros_cloud_type) {
        // 定义一个lambda函数
        const auto AddField = [&](const std::string& name, uint32_t offset, uint8_t data_type) {
            PointField point_field;
            point_field.count = 1u;
            point_field.datatype = data_type;
            point_field.name = name;
            point_field.offset = offset;
            cloud_ros.fields.push_back(point_field);
        };

        AddField("x", 0, PointField::FLOAT32);
        AddField("y", cloud_ros.fields.back().offset + sizeof(float), PointField::FLOAT32);
        AddField("z", cloud_ros.fields.back().offset + sizeof(float), PointField::FLOAT32);
        AddField("intensity", cloud_ros.fields.back().offset + sizeof(float), PointField::FLOAT32);
        AddField("time", cloud_ros.fields.back().offset + sizeof(uint32_t), PointField::UINT32);
        AddField("line", cloud_ros.fields.back().offset + sizeof(float), PointField::UINT8);
        AddField("tag", cloud_ros.fields.back().offset + sizeof(float), PointField::UINT8);
        cloud_ros.is_bigendian = false;
        cloud_ros.point_step = sizeof(float) * 4u + sizeof(uint32_t) + sizeof(uint8_t) * 2;
        cloud_ros.is_dense = true;
        init_ros_cloud_type = true;
    }
    if (system_has_init_) {
        // 点数的多少
        int point_size = livox_msg->point_num;
        // frame 和 time
        cloud_ros.header = livox_msg->header;
        cloud_ros.width = point_size;
        cloud_ros.height = 1;
        cloud_ros.row_step = point_size * cloud_ros.point_step;
        cloud_ros.data.resize(point_size * cloud_ros.point_step);
        // vector的首地址
        unsigned char* ptr = cloud_ros.data.data();
        for (size_t i = 0; i < point_size; ++i) {
            *(reinterpret_cast<float*>(ptr + cloud_ros.fields[0].offset)) = livox_msg->points[i].x;
            *(reinterpret_cast<float*>(ptr + cloud_ros.fields[1].offset)) = livox_msg->points[i].y;
            *(reinterpret_cast<float*>(ptr + cloud_ros.fields[2].offset)) = livox_msg->points[i].z;
            *(reinterpret_cast<float*>(ptr + cloud_ros.fields[3].offset)) =
                static_cast<float>(livox_msg->points[i].reflectivity);
            *(reinterpret_cast<uint32_t*>(ptr + cloud_ros.fields[4].offset)) = livox_msg->points[i].offset_time;
            *(reinterpret_cast<uint8_t*>(ptr + cloud_ros.fields[5].offset)) = livox_msg->points[i].line;
            *(reinterpret_cast<uint8_t*>(ptr + cloud_ros.fields[6].offset)) = livox_msg->points[i].tag;
            ptr += cloud_ros.point_step;
        }
        sensor_msgs::PointCloud2::Ptr cloud_ptr(new sensor_msgs::PointCloud2());
        *cloud_ptr = cloud_ros;
        std::lock_guard<std::mutex> lck(mtx_raw_cloud_queue);
        raw_cloud_queue_.push_back(cloud_ptr);
        // notify one
    }
}

void SystemManager::LidarStandarMsgCallback(const sensor_msgs::PointCloud2::Ptr& msg) {
    if (system_has_init_.load()) {
        std::lock_guard<std::mutex> lck(mtx_raw_cloud_queue);
        raw_cloud_queue_.push_back(msg);
        // notify one
    }
}
void SystemManager::IMUCallback(const sensor_msgs::Imu::Ptr& msg) {
}
void SystemManager::EncoderCallback(const nav_msgs::Odometry::Ptr& msg) {
}
void SystemManager::InitposeCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& init_pose_cov) {
}
bool SystemManager::SaveMap(slam_note::save_map::Request& req, slam_note::save_map::Response& resp) {
}
}  // namespace lio