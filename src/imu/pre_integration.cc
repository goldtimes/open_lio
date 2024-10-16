#include "imu/pre_integration.hh"

namespace pre_integration {
// using namespace lio;
PreIntegration::PreIntegration(ConfigParams &config_params) {}

void PreIntegration::IntegrateDataSegment(
    const std::vector<lio::IMUData> &imu_datas) {}

void PreIntegration::Integrate(const lio::IMUData &imu_data) {}

lio::NaviStateData
PreIntegration::Predict(const lio::NaviStateData &navi_state) {}
void PreIntegration::Reset() {}
} // namespace pre_integration