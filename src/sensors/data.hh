#pragma once

namespace lio {
using TimeStampUs = double;
struct DataBase {
    TimeStampUs timestamped_ = 0u;
    DataBase() = default;
    explicit DataBase(double time) : timestamped_(time) {
    }
};

}  // namespace lio