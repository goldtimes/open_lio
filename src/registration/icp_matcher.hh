#pragma once
#include <glog/logging.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "common/math_utils.hh"
#include "registration/registration_interface.hh"

namespace registration {

// icp典型的点到点的匹配方式
class ICPMatcher final : public RegistrationInterface {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    using PointType = PCLPointXYZI;
    using CloudType = PCLCloudXYZI;
    ICPMatcher(int max_iterations, int map_queue_size, double local_map_filter_size, double source_filter_size,
               double max_correspond_distance, double position_converage_thresh, double rotation_converage_thresh,
               double keyframe_position_thresh, double Keyframe_rotation_thresh, bool is_localization_mode);

    bool Match(const lio::PointCloudClusterPtr& cloud_cluster, Eigen::Matrix<double, 4, 4>& T);

   public:
   private:
    // icp的迭代次数
    int max_iterations_;
    // keyframes的队列的size
    int map_queue_size_;
    // 点到点的距离阈值
    double max_correspond_distance_;
    // 体素滤波
    double local_map_filter_size_;
    double source_filter_size_;
    // 匹配上的阈值
    double position_converage_thresh_;
    double rotation_converage_thresh_;
    // keyframe的提取阈值
    double keyframe_position_thresh_;
    double Keyframe_rotation_thresh_;
    // 是否收敛
    bool has_converage_;

    CloudType::Ptr local_map_ptr_;
    CloudType::Ptr source_cloud_ptr_;
    Eigen::Matrix<double, 4, 4> final_transformation_;
    pcl::KdTreeFLANN<PointType> kdtree_flann_{};

    bool is_localization_mode_;
};
};  // namespace registration