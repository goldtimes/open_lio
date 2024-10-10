#pragma once
#include <execution>
#include "common/data_type.hh"

namespace lidar_utils {
template <typename PointType>
inline void RemoveNanFromPointCloud(const pcl::PointCloud<PointType>& cloud_in, pcl::PointCloud<PointType>& cloud_out) {
    // 如果不是同一个点云
    if (&cloud_in != &cloud_out) {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
        cloud_out.sensor_origin_ = cloud_in.sensor_origin_;
        cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
    }
    // is_dense 为非零的点云
    if (cloud_in.is_dense) {
        cloud_out = cloud_in;
    } else {
        size_t j = 0;
        for (size_t i = 0; i < cloud_in.points.size(); ++i) {
            if (std::isnan(cloud_in.points[i].x) || std::isnan(cloud_in.points[i].y) ||
                std::isnan(cloud_in.points[i].z)) {
                continue;
            }
            cloud_out.points[j] = cloud_in.points[i];
            j++:
        }
        cloud_out.resize(j);
        cloud_out.height = 1;
        cloud_out.width = static_cast<uint32_t>(j);
        cloud_out.is_dense = true;
    }
}

}  // namespace lidar_utils