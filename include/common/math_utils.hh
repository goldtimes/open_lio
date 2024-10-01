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

}  // namespace math_utils