/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-10-16 23:37:07
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-10-17 00:12:24
 * @FilePath: /slam_ws/src/open_lio/src/imu/pre_integration.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include "common/math_utils.hh"
#include "sensors/imu.hh"
#include "sensors/navi_state.hh"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <deque>
#include <glog/logging.h>
#include <mutex>

namespace pre_integration {
class PreIntegration {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  struct ConfigParams {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  explicit PreIntegration(ConfigParams &config_params);

  void Integrate(const lio::IMUData &imu_data);
  void IntegrateDataSegment(const std::vector<lio::IMUData> &imu_datas);

  lio::NaviStateData Predict(const lio::NaviStateData &navi_state);
  void Reset();
  
  [[nodiscard]] const Eigen::Matrix<double, 9, 9> &Covariance() const;

  [[nodiscard]] const Eigen::Matrix3d &DeltaR() const;

  [[nodiscard]] const Eigen::Vector3d &DeltaV() const;

  [[nodiscard]] const Eigen::Vector3d &DeltaP() const;

  [[nodiscard]] const Eigen::Matrix3d &Get_dR_dbg() const;

  [[nodiscard]] const Eigen::Matrix3d &Get_dP_dbg() const;

  [[nodiscard]] const Eigen::Matrix3d &Get_dP_dba() const;

  [[nodiscard]] const Eigen::Matrix3d &Get_dV_dbg() const;

  [[nodiscard]] const Eigen::Matrix3d &Get_dV_dba() const;

  [[nodiscard]] const Eigen::Vector3d &GetAccBias() const;

  [[nodiscard]] const Eigen::Vector3d &GetGyroBias() const;

  [[nodiscard]] const Eigen::Vector3d &GetGravity() const;

  [[nodiscard]] const double &GetTotalIntegrationTime() const;

  [[nodiscard]] const uint64_t &GetStartIntegrationTimestamp() const;

  void SetGravity(const Eigen::Vector3d &gravity);

  void SetGyroAccBias(const Eigen::Vector3d &gyro_bias,
                      const Eigen::Vector3d &acc_bias);

  void SetGyroAccNoiseStd(const Eigen::Vector3d &gyro_std,
                          const Eigen::Vector3d &acc_std);

private:
  ConfigParams params_;

  Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_bias_ = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 6, 6> gyro_acc_noise_cov_ =
      Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix3d integration_cov_ = Eigen::Matrix3d::Zero();
  Eigen::Vector3d gravity_ = Eigen::Vector3d::Zero();

  // 预积分的变量
  Eigen::Matrix3d delta_R_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d delta_P_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d delta_V_ = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 9, 9> cov_ = Eigen::Matrix<double, 9, 9>::Identity();
  // 预积分对ba,bg的求导
  Eigen::Matrix3d dR_dbg_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dV_dbg_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dV_dba_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dP_dbg_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dV_dba_ = Eigen::Matrix3d::Zero();

  double total_time_; // 预积分的总时间
  uint64_t start_time_;
  lio::IMUData last_imu_data_;
  bool has_first_data_;
};
} // namespace pre_integration