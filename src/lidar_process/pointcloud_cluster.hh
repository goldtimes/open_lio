#pragma once
#include "common/data_type.hh"
#include "sensors/imu.hh"
#include "sensors/point_types.hh"

namespace lio {
struct PointCloudCluster {
    PointCloudCluster() {
        raw_cloud_.reset(new PCLCloudXYZIRT);
        ordered_cloud_.reset(new PCLCloudXYZIRT);
        corner_cloud_.reset(new PCLCloudXYZIRT);
        planar_cloud_.reset(new PCLCloudXYZIRT);
    }
    PointCloudCluster(const PointCloudCluster& other) noexcept {
        raw_cloud_ = other.raw_cloud_;
        ordered_cloud_ = other.ordered_cloud_;
        corner_cloud_ = other.corner_cloud_;
        planar_cloud_ = other.planar_cloud_;

        point_depth_vec_ = other.point_depth_vec_;
        point_col_index_vec_ = other.point_col_index_vec_;
        row_start_index_vec_ = other.row_start_index_vec_;
        row_end_index_vec_ = other.row_end_index_vec_;

        timestamped_ = other.timestamped_;
    }
    PointCloudCluster& operator=(const PointCloudCluster& other) {
        if (this == &other) {
            return *this;
        }
        raw_cloud_ = other.raw_cloud_;
        ordered_cloud_ = other.ordered_cloud_;
        corner_cloud_ = other.corner_cloud_;
        planar_cloud_ = other.planar_cloud_;

        point_depth_vec_ = other.point_depth_vec_;
        point_col_index_vec_ = other.point_col_index_vec_;
        row_start_index_vec_ = other.row_start_index_vec_;
        row_end_index_vec_ = other.row_end_index_vec_;

        timestamped_ = other.timestamped_;
        return *this;
    }

    PointCloudCluster(PointCloudCluster&& other) {
        raw_cloud_ = other.raw_cloud_;
        ordered_cloud_ = other.ordered_cloud_;
        corner_cloud_ = other.corner_cloud_;
        planar_cloud_ = other.planar_cloud_;

        point_depth_vec_ = other.point_depth_vec_;
        point_col_index_vec_ = other.point_col_index_vec_;
        row_start_index_vec_ = other.row_start_index_vec_;
        row_end_index_vec_ = other.row_end_index_vec_;

        timestamped_ = other.timestamped_;
    }

    void Clear() {
        raw_cloud_->clear();
        ordered_cloud_->clear();
        corner_cloud_->clear();
        planar_cloud_->clear();
    }

    PCLCloudXYZIRT::Ptr raw_cloud_;      // lidar frame
    PCLCloudXYZIRT::Ptr ordered_cloud_;  // 去畸变后的点云 imu_frame
    PCLCloudXYZIRT::Ptr corner_cloud_;   // 角点 imu_frame
    PCLCloudXYZIRT::Ptr planar_cloud_;   // 平面点 imu_frame
    uint64_t timestamped_ = 0;           // us

    std::vector<IMUData> imu_datas_;
    std::vector<float> point_depth_vec_;
    std::vector<int> row_start_index_vec_;
    std::vector<int> row_end_index_vec_;
    std::vector<int> point_col_index_vec_;
};

using PointCloudClusterPtr = std::shared_ptr<PointCloudCluster>;
}  // namespace lio