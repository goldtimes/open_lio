#include "system/SystemManager.hh"
#include <glog/logging.h>
#include "common/constant_variable.hh"
#include "common/ros_util.hh"
#include "lidar_process/lidar_model.hh"
#include "system/ConfigParams.hh"
namespace lio {
SystemManager::SystemManager(std::shared_ptr<ros::NodeHandle> nh) {
    nh_ = std::move(nh);
    InitConfigParams();
    InitLidarModel();
    InitRosPublishers();
    InitRosSubscribers();

    imu_data_searcher_ptr_ = std::make_shared<IMUDataSearcher>(ConfigParams::GetInstance().imu_data_buffer_size_);
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
    nh_->param<int>("imu_data_buffer_size", config.imu_data_buffer_size_, 2000);
    LOG(INFO) << "imu_data_buffer_size:" << config.imu_data_buffer_size_;
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
    nh_->param<bool>("imu/has_orientation", config.imu_config.has_orientation, false);
    LOG(INFO) << "imu/has_orientation:" << config.imu_config.has_orientation;
    // encoder config
    nh_->param<double>("encoder/encoder_position_noise", config.encoder_config.encoder_position_noise, 0.0001);
    nh_->param<double>("encoder/encoder_rotation_noise", config.encoder_config.encoder_rotation_noise, 0.05);
    LOG(INFO) << "encoder/encoder_position_noise:" << config.encoder_config.encoder_position_noise;
    LOG(INFO) << "encoder/encoder_rotation_noise:" << config.encoder_config.encoder_rotation_noise;
    // gravity
    nh_->param<double>("gravity", config.gravity_norm, 9.81);
    LOG(INFO) << "gravity_norm:" << config.gravity_norm;
    // T_LI;
    std::vector<double> T_lidar_to_imu;
    nh_->getParam("calibration/lidar_to_imu", T_lidar_to_imu);
    config.T_IL_ = Eigen::Map<Eigen::Matrix<double, 4, 4>, Eigen::RowMajor>(T_lidar_to_imu.data());
    // 前端参数
    // frontend config parameters
    nh_->param("frontend/registration_and_searcher_mode", config.front_config.registration_and_searcher_mode_,
               StringEmpty);
    nh_->param("frontend/feature/corner_thres", config.front_config.loam_feature_corner_thres_, FloatNaN);
    nh_->param("frontend/feature/planar_thres", config.front_config.loam_feature_planar_thres_, FloatNaN);
    nh_->param("frontend/feature/planar_voxel_filter_size", config.front_config.loam_feature_planar_voxel_filter_size_,
               FloatNaN);
    nh_->param("frontend/feature/corner_voxel_filter_size", config.front_config.loam_feature_corner_voxel_filter_size_,
               FloatNaN);
    nh_->param("frontend/registration/line_ratio_thres", config.front_config.registration_line_ratio_thres_, FloatNaN);
    nh_->param("frontend/registration/point_search_thres", config.front_config.registration_point_search_thres_,
               FloatNaN);
    nh_->param("frontend/registration/point_to_planar_thres", config.front_config.registration_point_to_planar_thres_,
               FloatNaN);
    nh_->param("frontend/registration/local_planar_map_size", config.front_config.registration_local_planar_map_size_,
               IntNaN);
    nh_->param("frontend/registration/local_corner_map_size", config.front_config.registration_local_corner_map_size_,
               IntNaN);
    nh_->param("frontend/registration/keyframe_delta_distance", config.front_config.registration_keyframe_delta_dist_,
               DoubleNaN);
    nh_->param("frontend/registration/keyframe_delta_rotation",
               config.front_config.registration_keyframe_delta_rotation_, DoubleNaN);
    nh_->param("frontend/registration/rotation_converge_thres",
               config.front_config.registration_rotation_converge_thres_, FloatNaN);
    nh_->param("frontend/registration/position_converge_thres",
               config.front_config.registration_position_converge_thres_, FloatNaN);
    nh_->param("frontend/registration/local_corner_voxel_filter_size",
               config.front_config.registration_local_corner_filter_size_, FloatNaN);
    nh_->param("frontend/registration/local_planar_voxel_filter_size",
               config.front_config.registration_local_planar_filter_size_, FloatNaN);
    nh_->param("frontend/registration/local_map_size", config.front_config.registration_local_map_size_, IntNaN);
    nh_->param("frontend/registration/local_map_cloud_filter_size",
               config.front_config.registration_local_map_cloud_filter_size_, FloatNaN);
    nh_->param("frontend/registration/source_cloud_filter_size",
               config.front_config.registration_source_cloud_filter_size_, FloatNaN);
    nh_->param("frontend/registration/optimization_iter_num", config.front_config.registration_opti_iter_num_, IntNaN);
    nh_->param("frontend/registration/ndt_voxel_size", config.front_config.registration_ndt_voxel_size_, DoubleNaN);
    nh_->param("frontend/registration/ndt_outlier_threshold", config.front_config.registration_ndt_outlier_threshold_,
               DoubleNaN);
    nh_->param("frontend/registration/ndt_min_points_in_voxel",
               config.front_config.registration_ndt_min_points_in_voxel_, IntNaN);
    nh_->param("frontend/registration/ndt_max_points_in_voxel",
               config.front_config.registration_ndt_max_points_in_voxel_, IntNaN);
    nh_->param("frontend/registration/ndt_min_effective_pts", config.front_config.registration_ndt_min_effective_pts_,
               IntNaN);
    nh_->param("frontend/registration/ndt_capacity", config.front_config.registration_ndt_capacity_, IntNaN);
    nh_->param("frontend/fusion_method", config.fusion_method_, StringEmpty);
    nh_->param("frontend/fusion_opti_iters", config.front_config.frontend_fusion_opti_iters_, IntNaN);
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
        size_t point_size = livox_msg->point_num;
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
        cv_preprogressing_.notify_one();
    }
}

void SystemManager::LidarStandarMsgCallback(const sensor_msgs::PointCloud2::Ptr& msg) {
    if (system_has_init_.load()) {
        std::lock_guard<std::mutex> lck(mtx_raw_cloud_queue);
        raw_cloud_queue_.push_back(msg);
        // notify one
        cv_preprogressing_.notify_one();
    }
}
void SystemManager::IMUCallback(const sensor_msgs::Imu::Ptr& msg) {
    // 无论什么松耦合\紧耦合都需要初始化
    static V3d init_mean_acc = V3d::Zero();  // acc的均值
    static V3d last_angular_vel = V3d::Zero();
    static Eigen::Quaterniond last_orientation;
    static TimeStampUs last_timestamped_;
    IMUData imu_data(RosTimeToUs(msg->header), RosV3dToEigen(msg->angular_velocity),
                     RosV3dToEigen(msg->linear_acceleration));
    /// 默认为紧耦合的方式
    // 未初始化
    if (!system_has_init_.load()) {
        const auto result = TryToInitIMU(imu_data, init_mean_acc);
        system_has_init_.store(result);
        if (system_has_init_.load()) {
            // 初始化完成, 设置朝向信息
            if (ConfigParams::GetInstance().imu_config.has_orientation) {
                imu_data.orientation_ = RosQuaternionToEigen(msg->orientation);
            } else {
                imu_data.orientation_ = Eigen::Quaterniond::Identity();
                last_angular_vel = imu_data.gyro_;
                last_orientation = Eigen::Quaterniond::Identity();
                last_timestamped_ = imu_data.timestamped_;
            }
            // 将这一一帧的imu保存起来
            imu_data.acc_ = imu_data.acc_ * ConfigParams::GetInstance().gravity_norm / init_mean_acc.norm();
            // cache imu
            imu_data_searcher_ptr_->CacheData(imu_data);
        }
        return;
    }
    // 初始化成功后
    imu_data.acc_ = imu_data.acc_ * ConfigParams::GetInstance().gravity_norm / init_mean_acc.norm();
    if (ConfigParams::GetInstance().imu_config.has_orientation) {
        imu_data.orientation_ = RosQuaternionToEigen(msg->orientation);
    } else {
        // 角速度*dt = 旋转量
        const Eigen::Quaterniond delta_q(
            math_utils::SO3Exp((last_angular_vel + imu_data.gyro_) * 0.5 * (imu_data.timestamped_ - last_timestamped_) *
                               kMicroseconds2Seconds));
        imu_data.orientation_ = last_orientation * delta_q;
        last_timestamped_ = imu_data.timestamped_;
        last_orientation = imu_data.orientation_;
        last_angular_vel = imu_data.gyro_;
    }
    // cache imu data
    imu_data_searcher_ptr_->CacheData(imu_data);
}

bool SystemManager::TryToInitIMU(const IMUData& imu_data, Eigen::Vector3d& init_acc) {
    // 计算零偏和方差
    static V3d mean_acc = V3d::Zero();
    static V3d mean_gyro = V3d::Zero();
    static V3d cov_acc = V3d::Zero();
    static V3d cov_gyro = V3d::Zero();
    static int N = 0;

    if (N == 0) {
        mean_acc = imu_data.acc_;
        mean_gyro = imu_data.gyro_;
        N++;
    } else {
        // 均值的更新和方差的计算公式不太一样
        // 更新均值
        const auto& acc = imu_data.acc_;
        const auto& gyro = imu_data.gyro_;
        mean_acc += (acc - mean_acc) / N;
        mean_gyro += (gyro - mean_gyro) / N;
        // cov = cov * (N - 1) / N + (new_value - mean) * (new_value - mean)T * (N - 1) / (N * N)
        // 更新方差
        cov_acc = cov_acc * (N - 1) / N + (acc - mean_acc).cwiseProduct(mean_acc - acc) * (N - 1) / (N * N);
        cov_gyro = cov_gyro * (N - 1) / N + (gyro - mean_gyro).cwiseProduct(mean_gyro - gyro) * (N - 1) / (N * N);
        N++;
    }
    init_acc = mean_acc;
    if (N > 300) {
        N = 0;
        mean_acc = V3d::Zero();
        mean_gyro = V3d::Zero();
        cov_acc = V3d::Zero();
        cov_gyro = V3d::Zero();
        LOG(WARNING) << "IMU Movement acceleration is too larger";
        return false;
    }
    if (N > 200 && cov_acc.norm() < 0.05 && cov_gyro.norm() < 0.01) {
        ConfigParams::GetInstance().gravity_vec =
            -mean_acc / mean_acc.norm() * ConfigParams::GetInstance().gravity_norm;
        LOG(INFO) << "mean acc:" << mean_acc.transpose() << ", mean gyro:" << mean_gyro.transpose();
        LOG(INFO) << "cov acc:" << cov_acc.transpose() << ", cov gyro:" << cov_gyro.transpose();
        return true;
    }
    return false;
}

void SystemManager::EncoderCallback(const nav_msgs::Odometry::Ptr& msg) {
}
void SystemManager::InitposeCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& init_pose_cov) {
}
bool SystemManager::SaveMap(slam_note::save_map::Request& req, slam_note::save_map::Response& resp) {
    return true;
}
}  // namespace lio