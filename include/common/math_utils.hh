#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <numeric>

#define kMicroseconds2Seconds (1.0 / 1e6)
namespace math_utils {
const double kD2R = M_PI / 180.0;
const double kR2D = 180.0 / M_PI;

// 传入float,double 的eigen的3维向量，然后计算反对称矩阵
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> SO3Hat(const Eigen::MatrixBase<Derived>& v) {
    eigen_assert(v.size() == 3u);

    Eigen::Matrix<typename Derived::Scalar, 3, 3> skew_mat;
    skew_mat.setZero();
    skew_mat(0, 1) = -v(2);
    skew_mat(0, 2) = v(1);
    skew_mat(1, 0) = v(2);
    skew_mat(1, 2) = -v(0);
    skew_mat(2, 0) = -v(1);
    skew_mat(2, 1) = v(0);
    return skew_mat;
}

// 角轴/旋转向量so3的SO3
// 利用罗德里格斯公式将旋转向量转换为旋转矩阵
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> SO3Exp(const Eigen::MatrixBase<Derived>& v) {
    eigen_assert(v.size() == 3u);
    Eigen::Matrix<typename Derived::Scalar, 3, 3> R = Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity();
    // 计算角轴的norm()
    typename Derived::Scalar theta = v.norm();
    if (theta > std::numeric_limits<typename Derived::Scalar>::epsilon()) {
        //归一化的角轴向量，代表方向
        Eigen::Matrix<typename Derived::Scalar, 3, 1> n = v.normalized();
        R = std::cos(theta) * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() +
            (typename Derived::Scalar(1.0) - std::cos(theta)) * n * n.transpose() + std::sin(theta) * SO3Hat(n);
        return R;
        //罗德里格斯公式
    } else {
        return R;
    }
}

// rotation to rpy
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1> RotationMatrixToRPY(const Eigen::MatrixBase<Derived>& R) {
    // 检查是否接近奇异值，例如90度的俯仰
    typename Derived::Scalar roll, pitch, yaw;
    if (std::abs(R(2, 0)) >= 1) {
        // 当俯仰接近90度时，roll和yaw将不再具有唯一解
        yaw = 0.0;
        if (R(2, 0) < 0) {  // pitch is 90 degrees
            pitch = M_PI / 2.0;
            roll = yaw + atan2(R(0, 1), R(0, 2));
        } else {  // pitch is -90 degrees
            pitch = -M_PI / 2.0;
            roll = -yaw + atan2(-R(0, 1), -R(0, 2));
        }
    } else {
        // 正常情况下计算roll, pitch, yaw
        roll = atan2(R(2, 1), R(2, 2));
        pitch = -asin(R(2, 0));
        yaw = atan2(R(1, 0), R(0, 0));
    }
    return {roll, pitch, yaw};
}

// template <typename Derived>
// inline Eigen::Matrix<typename Derived::Scalar, 3, 1> RotationMatrixToRPY(const Eigen::MatrixBase<Derived>& R) {
//     eigen_assert(R.rows() == 3u);
//     eigen_assert(R.cols() == 3u);

//     typename Derived::Scalar roll, pitch, yaw;
//     roll = std::atan2(R(2, 1), R(2, 2));
//     pitch = std::asin(-R(2, 0));
//     yaw = std::atan2(R(1, 0), R(0, 0));

//     return {roll, pitch, yaw};
// }
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> So3JacobianLeft(const Eigen::MatrixBase<Derived>& v) {
    eigen_assert(v.size() == 3u);

    const typename Derived::Scalar phi = v.norm();
    const typename Eigen::Matrix<typename Derived::Scalar, 3, 1> v_normlized = v.normalized();

    if (phi <= std::numeric_limits<typename Derived::Scalar>::epsilon()) {
        return Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity();
    } else {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> Jl;
        Jl = Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() * std::sin(phi) / phi +
             (typename Derived::Scalar(1.0) - std::sin(phi) / phi) * v_normlized * v_normlized.transpose() +
             (typename Derived::Scalar(1.0) - std::cos(phi)) / phi * SO3Hat(v_normlized);

        return Jl;
    }
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> So3JacobianRight(const Eigen::MatrixBase<Derived>& v) {
    eigen_assert(v.size() == 3u);

    const typename Derived::Scalar phi = v.norm();
    const typename Eigen::Matrix<typename Derived::Scalar, 3, 1> v_normlized = v.normalized();

    const Eigen::Matrix<typename Derived::Scalar, 3, 3> v_normlized_hat = SO3Hat(v_normlized);

    if (phi < std::numeric_limits<typename Derived::Scalar>::epsilon()) {
        return Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity();
    } else {
        const typename Derived::Scalar s2 = std::sin(phi / typename Derived::Scalar(2.0));
        const typename Derived::Scalar one_minus_cos = typename Derived::Scalar(2.0) * s2 * s2;
        const typename Derived::Scalar one_minus_sin_div_phi = (typename Derived::Scalar(1.0) - std::sin(phi) / phi);

        Eigen::Matrix<typename Derived::Scalar, 3, 3> Jr;
        Jr = Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - one_minus_cos / phi * v_normlized_hat +
             one_minus_sin_div_phi * v_normlized_hat * v_normlized_hat;

        return Jr;
    }
}

}  // namespace math_utils