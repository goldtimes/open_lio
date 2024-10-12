#pragma once
#include <pcl/filters/voxel_grid.h>
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
            j++;
        }
        cloud_out.resize(j);
        cloud_out.height = 1;
        cloud_out.width = static_cast<uint32_t>(j);
        cloud_out.is_dense = true;
    }
}

inline PCLCloudXYZI::Ptr VoxelGridCloud(const PCLCloudXYZI::Ptr& cloud, float voxel_size = 0.1) {
    pcl::VoxelGrid<PCLPointXYZI> filter;
    filter.setLeafSize(voxel_size, voxel_size, voxel_size);
    filter.setInputCloud(cloud);
    PCLCloudXYZI::Ptr out_cloud(new PCLCloudXYZI());
    filter.filter(*out_cloud);
    return out_cloud;
}

inline PCLPointXYZI TransformPoint(const PCLPointXYZI& point, const Eigen::Matrix3f& R, const Eigen::Vector3f& t) {
    Eigen::Vector3f point_eigen(point.x, point.y, point.z);
    Eigen::Vector3f trans_point = R * point_eigen + t;
    PCLPointXYZI result_point;
    result_point.x = trans_point(0);
    result_point.y = trans_point(1);
    result_point.z = trans_point(2);
    result_point.intensity = point.intensity;
    return result_point;
}

inline PCLCloudXYZI::Ptr TransformPointCloud(const PCLCloudXYZI::Ptr& cloud, const Eigen::Matrix4d& T) {
    const unsigned int N = cloud->size();
    const Eigen::Matrix3f R_temp = T.block<3, 3>(0, 0).cast<float>();
    const Eigen::Vector3f t_temp = T.block<3, 1>(0, 3).cast<float>();

    PCLCloudXYZI::Ptr cloud_trans(new PCLCloudXYZI);
    cloud_trans->resize(cloud->size());

    std::vector<size_t> indices(N);
    std::iota(indices.begin(), indices.end(), 0);
    std::for_each(std::execution::par, indices.begin(), indices.end(),
                  [&](size_t i) { cloud_trans->points[i] = TransformPoint(cloud->points[i], R_temp, t_temp); });
    return cloud_trans;
}

}  // namespace lidar_utils