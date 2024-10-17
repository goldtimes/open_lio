/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-10-16 23:37:07
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-10-17 00:25:09
 * @FilePath: /slam_ws/src/open_lio/src/imu/pre_integration.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <glog/logging.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <deque>
#include <mutex>
#include "common/math_utils.hh"
#include "sensors/imu.hh"
#include "sensors/navi_state.hh"

/**
 * @brief imu的预积分
 * 1. 推导imu积分的状态转移公式，协方差的更新公式
 * 2. 计算优化库所需要的dr_dbg,dp_dba, dp_dbg, dv_dbg, dv_dba
 */

namespace pre_integration {
class PreIntegration {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    struct ConfigParams {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d init_gyro_bias_ = Eigen::Vector3d::Zero();  // init gyro bias
        Eigen::Vector3d init_acc_bias_ = Eigen::Vector3d::Zero();   // init accelerometer bias
        Eigen::Vector3d gravity_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d gyro_noise_std_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d acc_noise_std_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d integration_noise_cov_ = Eigen::Vector3d::Zero();
    };
    explicit PreIntegration(ConfigParams &config_params);

    void Integrate(const lio::IMUData &imu_data);
    void IntegrateDataSegment(const std::vector<lio::IMUData> &imu_datas);

    lio::NaviStateData Predict(const lio::NaviStateData &navi_state);
    void Reset();

    [[nodiscard]] const Eigen::Matrix<double, 9, 9> &Covariance() const {
        return cov_;
    }

    [[nodiscard]] const Eigen::Matrix3d &DeltaR() const {
        return delta_R_;
    }

    [[nodiscard]] const Eigen::Vector3d &DeltaV() const {
        return delta_V_;
    }

    [[nodiscard]] const Eigen::Vector3d &DeltaP() const {
        return delta_P_;
    }

    [[nodiscard]] const Eigen::Matrix3d &Get_dR_dbg() const {
        return dR_dbg_;
    }

    [[nodiscard]] const Eigen::Matrix3d &Get_dP_dbg() const {
        return dP_dbg_;
    }

    [[nodiscard]] const Eigen::Matrix3d &Get_dP_dba() const {
        return dP_dbg_;
    }

    [[nodiscard]] const Eigen::Matrix3d &Get_dV_dbg() const {
        return dV_dbg_;
    }

    [[nodiscard]] const Eigen::Matrix3d &Get_dV_dba() const {
        return dV_dba_;
    }

    [[nodiscard]] const Eigen::Vector3d &GetAccBias() const {
        return acc_bias_;
    }

    [[nodiscard]] const Eigen::Vector3d &GetGyroBias() const {
        return gyro_bias_;
    }

    [[nodiscard]] const Eigen::Vector3d &GetGravity() const {
        return gravity_;
    }

    [[nodiscard]] const double &GetTotalIntegrationTime() const {
        return total_time_;
    }

    [[nodiscard]] const uint64_t &GetStartIntegrationTimestamp() const;

    void SetGravity(const Eigen::Vector3d &gravity) {
        gravity_ = gravity;
    }

    void SetGyroAccBias(const Eigen::Vector3d &gyro_bias, const Eigen::Vector3d &acc_bias) {
        acc_bias_ = acc_bias;
        gyro_bias_ = gyro_bias;
    }

    void SetGyroAccNoiseStd(const Eigen::Vector3d &gyro_std, const Eigen::Vector3d &acc_std) {
        gyro_acc_noise_cov_.diagonal().head<3>() = gyro_std.array().pow(2.0);
        gyro_acc_noise_cov_.diagonal().tail<3>() = acc_std.array().pow(2.0);
    }

   private:
    ConfigParams params_;

    Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_bias_ = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 6, 6> gyro_acc_noise_cov_ = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix3d integration_cov_ = Eigen::Matrix3d::Zero();
    Eigen::Vector3d gravity_ = Eigen::Vector3d::Zero();

    // 预积分的变量
    Eigen::Matrix3d delta_R_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d delta_P_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d delta_V_ = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 9, 9> cov_ = Eigen::Matrix<double, 9, 9>::Identity();
    // 预积分对ba,bg的求导
    Eigen::Matrix3d dR_dbg_ = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d dP_dbg_ = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d dP_dba_ = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d dV_dbg_ = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d dV_dba_ = Eigen::Matrix3d::Zero();
    // 预积分的间隔时间 一帧雷达之间的imu数据
    double total_time_;
    uint64_t start_time_;
    lio::IMUData last_imu_data_;
    bool has_first_data_;
};
}  // namespace pre_integration