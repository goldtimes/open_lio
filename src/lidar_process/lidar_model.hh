#pragma once
/**
 * 这是一个单例的模式
 * 1. 用delete的方式禁止构造函数
 */
#include <glog/logging.h>
#include <cmath>
#include <mutex>
#include <string>
#include "common/math_utils.hh"

namespace lio {
enum class LidarType { VELODYNE, OUSTER, ROBOSENSE, LEISHEN, MID360, AVIA, NONE };

class LidarModel {
   public:
    LidarType lidar_type = LidarType::NONE;

    static LidarModel* Instance(const std::string& type = "");
    // 默认构造函数
    LidarModel() = delete;
    // 拷贝构造函数
    LidarModel(const LidarModel& other) = delete;
    // 移动拷贝构造函数
    LidarModel(LidarModel&& other) = delete;
    // 复制构造函数
    LidarModel& operator=(const LidarModel& other) = delete;
    ~LidarModel();

    // 计算点所在的行号，要根据垂直分辨率结算
    [[nodiscard]] int RowIndex(float x, float y, float z) const {
        // 计算与z轴的夹角
        float xy = std::sqrt(x * x + y * y);
        // 计算所在的行
        const int row = static_cast<int>(std::round((atan2(z, xy) + lower_angle_) / v_res));
        return row;
    }
    // 计算col
    // 计算点对应的列(-pi对应的列索引是0, 0度对应的索引是水平采样数量的一半)
    [[nodiscard]] int ColIndex(float y, float x) const {
        int col = static_cast<int>(std::round(atan2(y, x) / h_res)) + h_scan_num_ / 2;
        if (col >= h_scan_num_) {
            col -= h_scan_num_;
        }
        return col;
    }

   public:
    float h_res;         // 水平分辨率
    float v_res;         // 垂直分辨率
    float lower_angle_;  // 垂直方向上的最小角度
    int h_scan_num_;     // 多少线的雷达
    int v_scan_num_;     // 每条线的扫描点数

   private:
    // 一个参数的构造函数并且私有化
    explicit LidarModel(const std::string& type = "");

   private:
    static LidarModel* instance;
    static std::string type_;
};
}  // namespace lio