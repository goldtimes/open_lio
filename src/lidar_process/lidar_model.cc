#include "lidar_process/lidar_model.hh"

namespace lio {
LidarModel* LidarModel::instance = nullptr;
std::string LidarModel::type_ = "NONE";

LidarModel::~LidarModel() {
    // 删除指针
    delete instance;
}
// 单例的获取
LidarModel* LidarModel::Instance(const std::string& type) {
    if (instance == nullptr && type.empty()) {
        LOG(ERROR) << "Lidar Type must given";
    }
    if (instance != nullptr && type.empty()) {
        LOG(ERROR) << "Lidar Model has been initialized";
    }
    if (instance == nullptr) {
        // 利用call_once来做到线程安全，否则多线程的时候，这样的单例会有问题
        static std::once_flag flag;
        std::call_once(flag, [&]() {
            type_ = type;
            instance = new (std::nothrow) LidarModel(type);
        });
    }
    return instance;
}

LidarModel::LidarModel(const std::string& type) {
    if (type == "None") {
        LOG(ERROR) << "must set lidar type";
        v_scan_num_ = 0;
        h_scan_num_ = 0;
        v_res = 0.0;
        h_res = 0.0;
        lower_angle_ = 0.0;
        lidar_type = LidarType::NONE;

    } else if (type == "Robosense") {
        v_scan_num_ = 16;
        h_scan_num_ = 1800;
        lidar_type = LidarType::ROBOSENSE;
        v_res = 2.0 * math_utils::kD2R;
        h_res = 0.2 * math_utils::kD2R;
        lower_angle_ = 15.0 * math_utils::kD2R;
    } else if (type == "LeiShen") {
        // 16 * 2000个点
        v_scan_num_ = 16;
        h_scan_num_ = 2000;
        // 角度转弧度
        v_res = 2.0 * math_utils::kD2R;
        h_res = 0.18 * math_utils::kD2R;
        lidar_type = LidarType::LEISHEN;
        lower_angle_ = 15.0 * math_utils::kD2R;
    } else if (type == "Velodyne_16") {
        // 16 * 2000个点
        v_scan_num_ = 16;
        h_scan_num_ = 2000;
        v_res = 2.0 * math_utils::kD2R;
        h_res = 0.2 * math_utils::kD2R;
        lower_angle_ = 15.0 * math_utils::kD2R;
        lidar_type = LidarType::VELODYNE;
    } else if (type == "Velodyne_64") {
        // 64 * 2000个点
        v_scan_num_ = 64;
        h_scan_num_ = 2000;
        v_res = 0.4 * math_utils::kD2R;
        h_res = 0.2 * math_utils::kD2R;
        lower_angle_ = 24.9 * math_utils::kD2R;
        lidar_type = LidarType::VELODYNE;
    } else if (type == "Ouster_128") {
        // 128 * 1024个点
        v_scan_num_ = 128;
        h_scan_num_ = 1024;
        v_res = (360.0 / 1024.0) * math_utils::kD2R;
        h_res = 0.35 * math_utils::kD2R;
        lower_angle_ = 24.9 * math_utils::kD2R;
        lidar_type = LidarType::OUSTER;
    } else if (type == "Mid360") {
        v_scan_num_ = -1;
        h_scan_num_ = -1;
        h_res = 0.0;
        v_res = 0.0;
        lower_angle_ = 0.0;
        lidar_type = LidarType::MID360;
    } else if (type == "Avia") {
        v_scan_num_ = -1;
        h_scan_num_ = -1;
        h_res = 0.0;
        v_res = 0.0;
        lower_angle_ = 0.0;
        lidar_type = LidarType::AVIA;
    } else {
        LOG(ERROR) << "unsupport lidar type";
    }
}
}  // namespace lio