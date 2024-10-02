#include "slam/preprocess.hh"
#include "system/ConfigParams.hh"

namespace lio {
PreProcessing::PreProcessing(std::shared_ptr<SystemManager> sys) {
    system_manager_ = sys;
    // lidar distortion corrector
    // cloud projector
    // feature extractor
    InitFilter();
}

void PreProcessing::InitFilter() {
    // const auto corner_size = ConfigParams::GetInstance().
}

void PreProcessing::Run() {
}

PCLCloudXYZIRT::Ptr PreProcessing::ConvertRosMessageToCloud(const sensor_msgs::PointCloud2Ptr& msg) {
}
// 计算每个点的时间偏移
void PreProcessing::ComputePointOffsetTime(const PCLCloudXYZIRT::Ptr& cloud, double lidar_rata) {
}
std::pair<float, float> PreProcessing::GetLidarPointMinMaxOffsetTime(const PCLCloudXYZIRT::Ptr& cloud) {
}
}  // namespace lio