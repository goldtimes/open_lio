#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
#include "sensors/data.hh"

namespace lio {
struct Odom : DataBase {
   public:
    Odom() = default;
    Odom(uint64_t us, double left_pulse, double right_pulse)
        : DataBase(us), left_pulse_(left_pulse), right_pulse_(right_pulse) {
    }
    double left_pulse_ = 0.0;
    double right_pulse_ = 0.0;
};
}  // namespace lio
using OdomPtr = std::shared_ptr<lio::Odom>;
