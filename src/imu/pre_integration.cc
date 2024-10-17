#include "imu/pre_integration.hh"
/// @brief 这里的预积分和高翔的slam中的预积分是一样的
namespace pre_integration {
// using namespace lio;
PreIntegration::PreIntegration(ConfigParams &config_params) {
    gyro_bias_ = config_params.init_gyro_bias_;
    acc_bias_ = config_params.init_acc_bias_;
    gravity_ = config_params.gravity_;
    integration_cov_.diagonal() = config_params.integration_noise_cov_;
    gyro_acc_noise_cov_.diagonal().head<3>() = config_params.gyro_noise_std_.array().pow(2.0);
    gyro_acc_noise_cov_.diagonal().tail<3>() = config_params.gyro_noise_std_.array().pow(2.0);
}

void PreIntegration::IntegrateDataSegment(const std::vector<lio::IMUData> &imu_datas) {
    for (const auto &imu_data : imu_datas) {
        Integrate(imu_data);
    }
}

// imu的状态传播,求到了delta量
void PreIntegration::Integrate(const lio::IMUData &curr_imu_data) {
    if (!has_first_data_) {
        has_first_data_ = true;
        last_imu_data_ = curr_imu_data;
        start_time_ = curr_imu_data.timestamped_;
        return;
    }
    // us - s
    const auto delta_t = static_cast<double>(curr_imu_data.timestamped_ - last_imu_data_.timestamped_) / 1.0e6;
    // 减去bias
    const Eigen::Vector3d acc_0 = last_imu_data_.acc_ - acc_bias_;
    const Eigen::Vector3d gyro_0 = last_imu_data_.gyro_ - gyro_0;

    const Eigen::Vector3d acc_1 = curr_imu_data.acc_ - acc_bias_;
    const Eigen::Vector3d gyro_1 = curr_imu_data.gyro_ - gyro_0;
    // 中值积分
    const Eigen::Vector3d acc = (acc_0 + acc_1) / 2;
    const Eigen::Vector3d gyro = (gyro_0 + gyro_1) / 2;
    // 计算旋转
    const Eigen::Matrix3d R_step = math_utils::SO3Exp(gyro * delta_t);
    const Eigen::Matrix3d acc_hat = math_utils::SO3Hat(acc);
    // 噪声转移矩阵
    Eigen::Matrix<double, 9, 9> A = Eigen::Matrix<double, 9, 9>::Identity();
    Eigen::Matrix<double, 9, 6> B = Eigen::Matrix<double, 9, 6>::Identity();

    // formula (13)
    A.block<3, 3>(0, 0) = R_step.transpose();
    A.block<3, 3>(3, 0) = -delta_R_ * acc_hat * delta_t;
    A.block<3, 3>(6, 0) = -0.5 * delta_R_ * acc_hat * delta_t * delta_t;
    A.block<3, 3>(6, 3) = delta_t * Eigen::Matrix3d::Identity();
    // 右雅可比矩阵 SO::Jr()
    // Sophus::SO3d::
    Eigen::Matrix3d Jr = math_utils::So3JacobianRight(gyro * delta_t);

    // formula (13)
    B.block<3, 3>(0, 0) = Jr * delta_t;
    B.block<3, 3>(3, 3) = delta_R_ * delta_t;
    B.block<3, 3>(6, 3) = 0.5 * delta_R_ * delta_t * delta_t;

    dP_dbg_ = dP_dbg_ + dV_dbg_ * delta_t - 0.5 * delta_R_ * acc_hat * dR_dbg_ * delta_t * delta_t;  // formula (21)
    dP_dba_ = dP_dba_ + dV_dba_ * delta_t - 0.5 * delta_R_ * delta_t * delta_t;                      // formula (22)
    dV_dbg_ = dV_dbg_ - delta_R_ * acc_hat * dR_dbg_ * delta_t;                                      // formula (19)
    dV_dba_ = dV_dba_ - delta_R_ * delta_t;                                                          // formula (20)
    dR_dbg_ = R_step.transpose() * dR_dbg_ - Jr * delta_t;                                           // formula (18)

    // 注意更新顺序是P, V, R. 如果顺序反了，将会带来较大的误差
    delta_P_ = delta_P_ + delta_V_ * delta_t + 0.5 * delta_R_ * acc * delta_t * delta_t;  // formula (3)
    delta_V_ = delta_V_ + delta_R_ * acc * delta_t;                                       // formula (2)
    delta_R_ = delta_R_ * R_step;                                                         // formula (1)

    // formula (14)
    cov_ = A * cov_ * A.transpose() + B * (gyro_acc_noise_cov_ / delta_t) * B.transpose();

    Eigen::Matrix3d i_cov = Eigen::Matrix3d::Identity() * integration_cov_ * delta_t;
    cov_.block<3, 3>(6, 6) += i_cov;
}

lio::NaviStateData PreIntegration::Predict(const lio::NaviStateData &navi_state) {
    // 根据delta量更新状态量
    const Eigen::Vector3d last_V = navi_state.V_;
    const Eigen::Vector3d last_P = navi_state.P_;
    const Eigen::Matrix3d last_R = navi_state.R_;

    lio::NaviStateData predict_navi_state;
    predict_navi_state.R_ = last_R * delta_R_;
    predict_navi_state.P_ =
        last_R * delta_P_ + last_P + last_V * total_time_ + 0.5 * gravity_ * total_time_ * total_time_;
    predict_navi_state.V_ = last_R * delta_V_ + last_V + gravity_ * total_time_;
}
void PreIntegration::Reset() {
    delta_R_ = Eigen::Matrix3d::Identity();
    delta_P_ = Eigen::Vector3d::Zero();
    delta_V_ = Eigen::Vector3d::Zero();
    cov_ = Eigen::Matrix<double, 9, 9>::Zero();
    dR_dbg_ = Eigen::Matrix3d::Zero();
    dP_dba_ = Eigen::Matrix3d::Zero();
    dP_dbg_ = Eigen::Matrix3d::Zero();
    dV_dba_ = Eigen::Matrix3d::Zero();
    dV_dbg_ = Eigen::Matrix3d::Zero();

    total_time_ = 0.0;
    start_time_ = 0.0;
    has_first_data_ = false;
}
}  // namespace pre_integration