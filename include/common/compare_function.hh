#pragma once

#include "common/data_type.hh"

namespace lio {
struct LessVec2i {
    bool operator()(const Eigen::Matrix<int, 2, 1>& v1, const Eigen::Matrix<int, 2, 1>& v2) const {
        return (v1[0] < v2[0]) || (v1[0] == v2[0] && v1[1] && v2[1]);
    }
};
struct LessVec3i {};
}  // namespace lio