#include "registration/icp_matcher.hh"
#include <iomanip>
#include "common/lidar_utils.hh"

namespace registration {
ICPMatcher::ICPMatcher(int max_iterations, int map_queue_size, double local_map_filter_size, double source_filter_size,
                       double max_correspond_distance, double position_converage_thresh,
                       double rotation_converage_thresh, double keyframe_position_thresh,
                       double Keyframe_rotation_thresh, bool is_localization_mode)
    : max_iterations_(max_iterations_),
      map_queue_size_(map_queue_size),
      local_map_filter_size_(local_map_filter_size),
      source_filter_size_(source_filter_size),
      max_correspond_distance_(max_correspond_distance),
      position_converage_thresh_(position_converage_thresh),
      rotation_converage_thresh_(rotation_converage_thresh),
      keyframe_position_thresh_(keyframe_position_thresh),
      Keyframe_rotation_thresh_(Keyframe_rotation_thresh),
      is_localization_mode_(is_localization_mode) {
    DLOG(INFO) << "max iteration: " << max_iterations_;
    DLOG(INFO) << "map_queue_size: " << map_queue_size_;
    DLOG(INFO) << "local_map_filter_size: " << local_map_filter_size_;
    DLOG(INFO) << "source_filter_size: " << source_filter_size_;
    DLOG(INFO) << "max_correspond_distance: " << max_correspond_distance_;
    DLOG(INFO) << "position_converage_thresh: " << position_converage_thresh_;
    DLOG(INFO) << "rotation_converage_thresh: " << rotation_converage_thresh_;
    DLOG(INFO) << "keyframe_position_thresh: " << keyframe_position_thresh_;
    DLOG(INFO) << "Keyframe_rotation_thresh: " << Keyframe_rotation_thresh_;
    DLOG(INFO) << "is_localization_mode: " << is_localization_mode_;
}

// 点到点的icp,手写高斯牛顿方法
bool ICPMatcher::Match(const lio::PointCloudClusterPtr& cloud_cluster, Eigen::Matrix<double, 4, 4>& T) {
    has_converage_ = false;
    // 降采样
    source_cloud_ptr_ = lidar_utils::VoxelGridCloud(cloud_cluster->ordered_cloud_, source_filter_size_);
    CloudType::Ptr transformed_cloud_ptr(new CloudType);
    Eigen::Matrix4d T_temp = T;
    for (size_t iteration = 0; iteration < max_iterations_; ++iteration) {
        transformed_cloud_ptr = lidar_utils::TransformPointCloud(source_cloud_ptr_, T_temp);
        Eigen::Matrix<double, 6, 6> Hessian = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> B = Eigen::Matrix<double, 6, 1>::Zero();
        size_t cloud_size = transformed_cloud_ptr->size();
        // 每个点构成的hessian矩阵
        std::vector<Eigen::Matrix<double, 6, 6>> H_all(cloud_size);
        std::vector<Eigen::Matrix<double, 6, 1>> b_all(cloud_size);
        // 残差
        std::vector<Eigen::Matrix<double, 3, 1>> error_vec(cloud_size);
        // 有效的点
        std::vector<bool> effect_pts(cloud_size, false);
        std::vector<size_t> indices(cloud_size);
        std::iota(indices.begin(), indices.end(), 0);

        std::for_each(std::execution::par, indices.begin(), indices.end(), [&](const size_t index) {
            const PointType& orign_point = source_cloud_ptr_->points[index];
            const PointType& transed_point = transformed_cloud_ptr->points[index];
            std::vector<float> dist;
            std::vector<int> ids;
            kdtree_flann_.nearestKSearch(transed_point, 1, ids, dist);
            if (dist.front() > max_correspond_distance_) {
                // 点到点的距离大于阈值
                return;
            }
            // 计算残差
            // point in world
            Eigen::Matrix<double, 3, 1> nearest_point =
                local_map_ptr_->points[ids.front()].getVector3fMap().cast<double>();
            // transformed point
            Eigen::Matrix<double, 3, 1> transed_point_eigen = transed_point.getVector3fMap().cast<double>();
            Eigen::Matrix<double, 3, 1> origin_point_eigen = orign_point.getVector3fMap().cast<double>();
            // T * p_i - p_j
            Eigen::Matrix<double, 3, 1> error = transed_point_eigen - nearest_point;
            // 于是对平移和旋转求雅克比矩阵
            Eigen::Matrix<double, 3, 6> Jacobian = Eigen::Matrix<double, 3, 6>::Zero();
            // 对平移求雅克比矩阵
            Jacobian.leftCols(3) = Eigen::Matrix<double, 3, 3>::Identity();
            // 对旋转求雅克比矩阵，这个用李代数的BCH可以很快计算得到
            Jacobian.rightCols(3) = -T_temp.block<3, 3>(0, 0) * math_utils::SO3Hat(origin_point_eigen);
            // H 矩阵
            Eigen::Matrix<double, 6, 6> H = Jacobian.transpose() * Jacobian;
            Eigen::Matrix<double, 6, 1> B = -Jacobian.transpose() * error;
            H_all[index] = H;
            b_all[index] = B;
            effect_pts[index] = true;
            error_vec[index] = error;
        });
        // 计算所有残差和有效的点
        double total_error;
        int effective_num;
        for (size_t i = 0; i < effect_pts.size(); ++i) {
            if (!effect_pts[i]) {
                continue;
            }
            effective_num++;
            Hessian += H_all[i];
            B += b_all[i];
            total_error += error_vec[i].norm();
        }
        if (!Hessian.determinant()) {
            // 下次迭代
            continue;
        }
        // 求delta_x;
        Eigen::Matrix<double, 6, 1> delta_x = Hessian.inverse() * B;
        // 更新T_temp
        T_temp.block<3, 3>(0, 0) *= math_utils::SO3Exp(delta_x.tail(3)).matrix();
        T_temp.block<3, 1>(0, 3) += delta_x.head<3>();

        // 判断是否收敛
        if (delta_x.head(3).norm() < position_converage_thresh_ &&
            delta_x.tail(3).norm() < rotation_converage_thresh_) {
            DLOG(INFO) << "num iter= " << std::setw(2) << iteration << " total point size= " << std::setprecision(5)
                       << cloud_size << " total res= " << std::setprecision(5) << total_error
                       << " mean res= " << std::setprecision(5) << total_error / static_cast<double>(effective_num)
                       << " valid point= " << effective_num;

            has_converage_ = true;
            break;
        }
    }
    final_transformation_ = T_temp;
    if (has_converage_ && IsNeedAddCloud(final_transformation_) && !is_localization_mode_) {
        // 根据距离是否更新keyframe
        CloudType::Ptr transformed_cloud(new CloudType);
        transformed_cloud = lidar_utils::TransformPointCloud(source_cloud_ptr_, final_transformation_);
        AddCloudToLocalMap({*transformed_cloud});
    } else if (!has_converage_) {
        LOG(ERROR) << "icp optimized match failed!\n";
    }
    return has_converage_;
}

void ICPMatcher::AddCloudToLocalMap(std::initializer_list<PCLCloudXYZI> cloud_list) {
    const auto new_cloud = (cloud_list.begin())->makeShared();
    if (is_localization_mode_) {
        *local_map_ptr_ = *new_cloud;
    } else {
        cloud_queue_.push_back(new_cloud);
        if (cloud_queue_.size() > map_queue_size_) {
            cloud_queue_.pop_front();
        }
        local_map_ptr_->clear();
        for (const auto& cloud : cloud_queue_) {
            *local_map_ptr_ += *cloud;
        }
        local_map_ptr_ = lidar_utils::VoxelGridCloud(local_map_ptr_, local_map_filter_size_);
        kdtree_flann_.setInputCloud(local_map_ptr_);
    }
}

// 返回值不可忽略且不可以修改，获取配准分数
[[nodiscard]] float ICPMatcher::GetFitnessScore(float max_range) const {
    float score = 0;
    // 将源点云通过T矩阵转换后，计算分数
    CloudType::Ptr transformed_cloud_ptr(new CloudType);
    transformed_cloud_ptr = lidar_utils::TransformPointCloud(source_cloud_ptr_, final_transformation_);

    std::vector<int> nn_indices;
    std::vector<float> nn_dist;

    int nr = 0;
    for (size_t i = 0; i < transformed_cloud_ptr->size(); ++i) {
        kdtree_flann_.nearestKSearch(transformed_cloud_ptr->points[i], 1, nn_indices, nn_dist);
        if (nn_dist.front() < max_range) {
            score += nn_dist.front();
            nr++;
        }
    }
    if (nr > 0) {
        return score / static_cast<float>(nr);
    } else {
        return -1;
    }
}

bool ICPMatcher::IsNeedAddCloud(const Eigen::Matrix<double, 4, 4>& T) {
    // 初始化一次
    static Eigen::Matrix<double, 4, 4> last_T = T;
    // 计算两帧的距离+
    Eigen::Matrix3d R_delta = last_T.block<3, 3>(0, 0).inverse() * T.block<3, 3>(0, 0);
    // matrix to rpy
    Eigen::Matrix<double, 3, 1> euler_delta = math_utils::RotationMatrixToRPY(R_delta);
    Eigen::Matrix<double, 3, 1> t_delta = last_T.block<3, 1>(0, 3) - T.block<3, 1>(0, 3);
    if (t_delta.norm() > keyframe_position_thresh_ || euler_delta.norm() > Keyframe_rotation_thresh_) {
        last_T = T;
        return true;
    } else {
        return false;
    }
}

}  // namespace registration