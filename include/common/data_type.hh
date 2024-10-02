#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <vector>
#include "sensors/point_types.hh"

// namespace lio {
// using TimeStampUs = uint64_t;

typedef typename pcl::PointXYZ PCLPointXYZ;
typedef typename pcl::PointXYZI PCLPointXYZI;

// using PointXYZI = pcl::PointXYZI;

using PCLCloudXYZ = pcl::PointCloud<PCLPointXYZ>;
using PCLCloudXYZI = pcl::PointCloud<PCLPointXYZI>;
using PCLCloudXYZIRT = pcl::PointCloud<PointXYZIRT>;

// using V3d = Eigen::Vector3d;

// }  // namespace lio