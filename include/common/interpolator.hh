#pragma once

#include <glog/logging.h>
#include <Eigen/Dense>

namespace lio {
// 线性插值和球形插值的算法
class Interpolator {
   public:
    Interpolator() = delete;
    Interpolator(Interpolator&) = delete;
    ~Interpolator() = delete;

    /**
     * @brief 四元素的球型插值
     */
    template <typename Derived>
    static Derived InterpolateQuaternionSlerp(const Eigen::QuaternionBase<Derived>& q0,
                                              const Eigen::QuaternionBase<Derived>& q1, uint64_t time_0,
                                              uint64_t time_1, uint64_t interpolate_time) {
        // time1 > time0
        CHECK_GT(time_1, time_0);
        CHECK_GE(interpolate_time, time_0);  // >=
        CHECK_LE(interpolate_time, time_1);
        typename Derived::Scalar temp_t = static_cast<typename Derived::Scalar>(interpolate_time - time_0) /
                                          static_cast<typename Derived::Scalar>(time_1 - time_0);
        typename Derived::Scalar one =
            typename Derived::Scalar(1) - std::numeric_limits<typename Derived::Scalar>::epsilon();
        typename Derived::Scalar d = q0.template dot(q1);
        typename Derived::Scalar abs_d = std::abs(d);
        typename Derived::Scalar scale_0;
        typename Derived::Scalar scale_1;

        if (abs_d >= one) {
            scale_0 = typename Derived::Scalar(1) - temp_t;
            scale_1 = temp_t;
        } else {
            typename Derived::Scalar theta = std::acos(abs_d);
            typename Derived::Scalar sin_theta = std::sin(theta);
            scale_0 = std::sin((typename Derived::Scalar(1) - temp_t) * theta) / sin_theta;
            scale_1 = std::sin(temp_t * theta) / sin_theta;
        }
        if (d < typename Derived::Scalar(0)) {
            scale_1 = -scale_1;
        }
        Derived q;
        q = scale_0 * q0.coeffs() + scale_1 * q1.coeffs();
        return q;
    }

    template <typename Derived>
    static Derived InterpolateVectorLerp(const Eigen::MatrixBase<Derived>& v_0, const Eigen::MatrixBase<Derived>& v_1,
                                         uint64_t t_0, uint64_t t_1, uint64_t t) {
        // time1 > time0
        // need glog
        CHECK_GT(t_1, t);
        CHECK_GE(t, t_0);  // >=
        CHECK_LE(t, t_1);

        typename Derived::Scalar temp_t =
            static_cast<typename Derived::Scalar>(t - t_0) / static_cast<typename Derived::Scalar>(t_1 - t_0);
        return ((typename Derived::Scalar(1.0)) - temp_t) * v_0 + temp_t * v_1;
    }
};
}  // namespace lio