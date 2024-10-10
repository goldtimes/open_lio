#include "slam/preprocess.hh"
#include <pcl_conversions/pcl_conversions.h>
#include "common/constant_variable.hh"
#include "common/lidar_utils.hh"
#include "imu/imu_interpolator.hh"
#include "lidar_process/lidar_distortion_corrector.hh"
#include "lidar_process/pointcloud_cluster.hh"
#include "system/ConfigParams.hh"
namespace lio {
PreProcessing::PreProcessing(std::shared_ptr<SystemManager> sys) {
    system_manager_ = sys;
    // lidar distortion corrector
    lidar_distortion_corrector_ptr_ = std::make_shared<LidarDistortionCorrector>(ConfigParams::GetInstance().T_IL_);
    // cloud projector

    // feature extractor
    InitFilter();
}

void PreProcessing::InitFilter() {
    const auto corner_size = ConfigParams::GetInstance().front_config.loam_feature_corner_voxel_filter_size_;
    const auto plannar_size = ConfigParams::GetInstance().front_config.loam_feature_planar_voxel_filter_size_;
    corner_voxel_filter_.setLeafSize(corner_size, corner_size, corner_size);
    plannar_voxel_filter_.setLeafSize(plannar_size, plannar_size, plannar_size);
}

void PreProcessing::Run() {
    LOG(INFO) << "\033[1;32m----> PreProcessing Started.\033[0m";
    // 时间同步
    while (ros::ok()) {
        // 等待system_manager通知
        sensor_msgs::PointCloud2Ptr raw_cloud_ros;

        static bool imu_data_block = false;
        static TimeStampUs last_cloud_timestamped = std::numeric_limits<TimeStampUs>::max();
        {
            std::unique_lock<std::mutex> lck(system_manager_->mtx_raw_cloud_queue);
            while (system_manager_->raw_cloud_queue_.empty() || imu_data_block) {
                if (!ros::ok()) {
                    return;
                }
                imu_data_block = false;
                // 雷达队列为空的时候，等待在这里，直到收到通知
                system_manager_->cv_preprogressing_.wait(lck);
            }
            // 有雷达数据
            raw_cloud_ros = system_manager_->raw_cloud_queue_.front();
        }
        // 点云为空
        if (raw_cloud_ros) {
            continue;
        }
        PCLCloudXYZIRT::Ptr raw_cloud_ptr_;
        // ros to PCLCloudXYZIRT
        if (temp_cloud_ptr_) {
            // imu延迟时，会等到imu到达这帧雷达消息时刻
            raw_cloud_ptr_ = temp_cloud_ptr_;
        } else {
            raw_cloud_ptr_ = ConvertRosMessageToCloud(raw_cloud_ros);
        }
        // 雷达当前时间\雷达帧首，帧尾时间
        const auto min_max_offset_time = GetLidarPointMinMaxOffsetTime(raw_cloud_ptr_);
        TimeStampUs curr_cloud_timestamped = raw_cloud_ptr_->header.stamp;

        uint64_t cloud_start_time = static_cast<uint64_t>(static_cast<int64_t>(raw_cloud_ptr_->header.stamp) +
                                                          static_cast<int64_t>(min_max_offset_time.first * 1.0e-6));
        uint64_t cloud_end_time = static_cast<uint64_t>(static_cast<int64_t>(raw_cloud_ptr_->header.stamp) +
                                                        static_cast<int64_t>(min_max_offset_time.second * 1.0e-6));
        // 因为不同雷达类型的时间戳是帧首还是帧尾
        if (curr_cloud_timestamped < cloud_start_time) {
            cloud_start_time = curr_cloud_timestamped;
        } else if (curr_cloud_timestamped > cloud_end_time) {
            cloud_end_time = curr_cloud_timestamped;
        }
        DLOG(INFO) << std::setprecision(15) << "last   cloud  timestamp: "
                   << static_cast<double>(last_cloud_timestamped) * kMicroseconds2Seconds;
        DLOG(INFO) << std::setprecision(15) << "current cloud timestamp: "
                   << static_cast<double>(curr_cloud_timestamped) * kMicroseconds2Seconds;
        DLOG(INFO) << std::setprecision(15) << "  cloud start timestamp: "
                   << static_cast<double>(curr_cloud_timestamped) * kMicroseconds2Seconds;
        DLOG(INFO) << std::setprecision(15)
                   << "  cloud  end  timestamp: " << static_cast<double>(cloud_end_time) * kMicroseconds2Seconds;
        if (raw_cloud_ptr_->empty()) {
            // 如果雷达为空
            temp_cloud_ptr_ = nullptr;
            // 加锁
            std::unique_lock<std::mutex> lck(system_manager_->mtx_raw_cloud_queue);
            system_manager_->raw_cloud_queue_.pop_front();
            LOG(WARNING) << "Lidar cloud data empty";
            continue;
        }
        // 雷达数据大于imu最早的数据
        const auto oldest_imu_data = system_manager_->imu_data_searcher_ptr_->OldestData();
        if (!oldest_imu_data.has_value() || oldest_imu_data.value().timestamped_ > cloud_start_time) {
            // 加锁
            std::unique_lock<std::mutex> lck(system_manager_->mtx_raw_cloud_queue);
            system_manager_->raw_cloud_queue_.pop_front();
            temp_cloud_ptr_ = nullptr;
            // lidar的数据延迟了
            LOG(WARNING) << "Lidar data queue may be blocked";
            continue;
        }
        // 雷达数据要大于imu最新的数据，imu_block
        const auto latest_imu_data = system_manager_->imu_data_searcher_ptr_->LatestData();
        if (!latest_imu_data.has_value() || latest_imu_data.value().timestamped_ < cloud_end_time) {
            temp_cloud_ptr_ = raw_cloud_ptr_;
            // imu延迟了，于是让循环阻塞
            imu_data_block = true;
            continue;
        }
        // 数据都正常，于是乎队列中删除这帧雷达
        {
            std::unique_lock<std::mutex> lck(system_manager_->mtx_raw_cloud_queue);
            system_manager_->raw_cloud_queue_.pop_front();
            temp_cloud_ptr_ = nullptr;
        }
        // 找到雷达帧之间的imu数据
        const auto imu_data_vector =
            system_manager_->imu_data_searcher_ptr_->GetDataSegment(cloud_start_time, cloud_end_time);
        auto imu_data_searcher = std::make_shared<DataSearcherIMU>(imu_data_vector);

        // 去畸变
        lidar_distortion_corrector_ptr_->SetDataSearcher(imu_data_searcher);
        lidar_distortion_corrector_ptr_->SetRefTime(curr_cloud_timestamped);

        auto cloud_cluster_ptr = std::make_shared<PointCloudCluster>();
        cloud_cluster_ptr->raw_cloud_ = raw_cloud_ptr_;
        cloud_cluster_ptr->timestamped_ = curr_cloud_timestamped;

        // get imu datas
        if (last_cloud_timestamped == std::numeric_limits<double>::max()) {
            // 第一帧
            IMUData data_l, data_r, inter_imu_data;
            system_manager_->imu_data_searcher_ptr_->SearchNearestTwoData(curr_cloud_timestamped, data_l, data_r);
            inter_imu_data = imu::IMUInterpolator(data_l, data_r, curr_cloud_timestamped);
            cloud_cluster_ptr->imu_datas_.emplace_back(inter_imu_data);
        } else {
            CHECK_GT(curr_cloud_timestamped, last_cloud_timestamped);
            cloud_cluster_ptr->imu_datas_ =
                system_manager_->imu_data_searcher_ptr_->GetDataSegment(last_cloud_timestamped, curr_cloud_timestamped);
        }
        DLOG(INFO) << "cloud cluster imu data size: " << cloud_cluster_ptr->imu_datas_.size();

        last_cloud_timestamped = curr_cloud_timestamped;
        // 不同的雷达里程计类型，做不同的处理
        if (ConfigParams::GetInstance().front_config.registration_and_searcher_mode_ == kPointToPlane_IVOX ||
            ConfigParams::GetInstance().front_config.registration_and_searcher_mode_ == kPointToPlane_KdTree ||
            ConfigParams::GetInstance().front_config.registration_and_searcher_mode_ == kIcpOptimized ||
            ConfigParams::GetInstance().front_config.registration_and_searcher_mode_ == kIncrementalNDT) {
            // 分配空间
            cloud_cluster_ptr->ordered_cloud_->points.reserve(raw_cloud_ptr_->size());
            cloud_cluster_ptr->planar_cloud_->points.reserve(raw_cloud_ptr_->size());
            for (size_t i = 0; i < raw_cloud_ptr_->size(); ++i) {
                // 过滤掉近的点和远的点
                Eigen::Vector3d point;
                point[0] = cloud_cluster_ptr->raw_cloud_->points[i].x;
                point[1] = cloud_cluster_ptr->raw_cloud_->points[i].y;
                point[2] = cloud_cluster_ptr->raw_cloud_->points[i].z;
                // point.intensity = cloud_cluster_ptr->raw_cloud_->points[i].intensity;
                double depth = std::sqrt(point[0] * point[0] + point[1] * point[1] + point[2] * point[2]);
                if (depth < ConfigParams::GetInstance().lidar_config.lidar_min_dist ||
                    depth > ConfigParams::GetInstance().lidar_config.lidar_max_dist) {
                    continue;
                }
                // 去畸变
                float relative_time = cloud_cluster_ptr->raw_cloud_->points[i].time;
                Eigen::Vector3f undistor_point;
                if (!lidar_distortion_corrector_ptr_->UnidstortPoint(point.cast<float>(), undistor_point,
                                                                     relative_time)) {
                    continue;
                }
                PCLPointXYZI point_xyzi;
                point_xyzi.x = undistor_point.x();
                point_xyzi.y = undistor_point.y();
                point_xyzi.z = undistor_point.z();
                point_xyzi.intensity = cloud_cluster_ptr->raw_cloud_->points[i].intensity;
                // 降采样
                if (i % ConfigParams::GetInstance().lidar_config.lidar_jump_span == 0) {
                    cloud_cluster_ptr->planar_cloud_->push_back(point_xyzi);
                }
                cloud_cluster_ptr->ordered_cloud_->push_back(point_xyzi);
            }
        } else if (ConfigParams::GetInstance().front_config.registration_and_searcher_mode_ == kLoamFull_KdTree) {
            // 特征提取
        }

        {
            // 加锁 放到system_manager的队列中
            std::lock_guard<std::mutex> lck(system_manager_->mtx_cloud_cluster_queue_);
            system_manager_->cloud_cluster_queue_.emplace_back(cloud_cluster_ptr);
            if (system_manager_->slam_mode_ == SLAM_MODE::MAPPING) {
                // 通知建图前端
                system_manager_->cv_frontend_.notify_one();
            } else if (system_manager_->slam_mode_ == SLAM_MODE::LOCALIZATION) {
                // 通知定位程序开始处理
                system_manager_->cv_localization_.notify_one();
            } else {
                LOG(ERROR) << "SLAM mode error";
            }
        }
        lidar_distortion_corrector_ptr_->SetDataSearcher(nullptr);
    }
}

PCLCloudXYZIRT::Ptr PreProcessing::ConvertRosMessageToCloud(const sensor_msgs::PointCloud2Ptr& msg) {
    if (LidarModel::Instance()->lidar_type == LidarType::VELODYNE) {
        pcl::PointCloud<VelodynePointXYZIRT> cloud_velodyne;
        pcl::fromROSMsg(*msg, cloud_velodyne);
        if (!cloud_velodyne.is_dense) {
            // 非稠密的点云
            lidar_utils::RemoveNanFromPointCloud(cloud_velodyne, cloud_velodyne);
        }
        const size_t point_size = cloud_velodyne.size();
        PCLCloudXYZIRT::Ptr cloud_ptr(new PCLCloudXYZIRT);
        cloud_ptr->resize(point_size);
        // 创建indics是为了在for_each中遍历cloud_velodyne
        std::vector<size_t> indics(point_size);
        // 0 ~ point_size;
        std::iota(indics.begin(), indics.end(), 0);
        std::for_each(std::execution::par, indics.begin(), indics.end(), [&](const size_t& index) {
            // 创建一个点
            PointXYZIRT point;
            point.x = cloud_velodyne[index].x;
            point.y = cloud_velodyne[index].y;
            point.z = cloud_velodyne[index].z;
            point.intensity = cloud_velodyne[index].intensity;
            point.ring = static_cast<uint8_t>(cloud_velodyne[index].ring);
            // us->s
            point.time = static_cast<float>(cloud_velodyne[index].time *
                                            ConfigParams::GetInstance().lidar_config.lidar_point_time_scale);
            cloud_ptr->at(index) = point;
        });
        cloud_ptr->header = cloud_velodyne.header;
        cloud_ptr->is_dense = true;
        if (cloud_ptr->points.back().time <= 0.0f) {
            ComputePointOffsetTime(cloud_ptr, 10.0);
        }
        return cloud_ptr;

    } else if (LidarModel::Instance()->lidar_type == LidarType::OUSTER) {
        pcl::PointCloud<VelodynePointXYZIRT> cloud_ouster;
        pcl::fromROSMsg(*msg, cloud_ouster);
        if (!cloud_ouster.is_dense) {
            // 非稠密的点云
            lidar_utils::RemoveNanFromPointCloud(cloud_ouster, cloud_ouster);
        }
        const size_t point_size = cloud_ouster.size();
        PCLCloudXYZIRT::Ptr cloud_ptr(new PCLCloudXYZIRT);
        cloud_ptr->resize(point_size);
        // 创建indics是为了在for_each中遍历cloud_velodyne
        std::vector<size_t> indics(point_size);
        // 0 ~ point_size;
        std::iota(indics.begin(), indics.end(), 0);
        std::for_each(std::execution::par, indics.begin(), indics.end(), [&](const size_t& index) {
            // 创建一个点
            PointXYZIRT point;
            point.x = cloud_ouster[index].x;
            point.y = cloud_ouster[index].y;
            point.z = cloud_ouster[index].z;
            point.intensity = cloud_ouster[index].intensity;
            point.ring = static_cast<uint8_t>(cloud_ouster[index].ring);
            // us->s
            point.time = static_cast<float>(cloud_ouster[index].time *
                                            ConfigParams::GetInstance().lidar_config.lidar_point_time_scale);
            cloud_ptr->at(index) = point;
        });
        cloud_ptr->header = cloud_ouster.header;
        cloud_ptr->is_dense = true;
        return cloud_ptr;

    } else if (LidarModel::Instance()->lidar_type == LidarType::LEISHEN) {
        pcl::PointCloud<VelodynePointXYZIRT> cloud_leishen;
        pcl::fromROSMsg(*msg, cloud_leishen);
        if (!cloud_leishen.is_dense) {
            // 非稠密的点云
            lidar_utils::RemoveNanFromPointCloud(cloud_leishen, cloud_leishen);
        }
        const size_t point_size = cloud_leishen.size();
        PCLCloudXYZIRT::Ptr cloud_ptr(new PCLCloudXYZIRT);
        cloud_ptr->resize(point_size);
        // 创建indics是为了在for_each中遍历cloud_velodyne
        std::vector<size_t> indics(point_size);
        // 0 ~ point_size;
        std::iota(indics.begin(), indics.end(), 0);
        std::for_each(std::execution::par, indics.begin(), indics.end(), [&](const size_t& index) {
            // 创建一个点
            PointXYZIRT point;
            point.x = cloud_leishen[index].x;
            point.y = cloud_leishen[index].y;
            point.z = cloud_leishen[index].z;
            point.intensity = cloud_leishen[index].intensity;
            point.ring = static_cast<uint8_t>(cloud_leishen[index].ring);
            // us->s
            point.time = static_cast<float>(cloud_leishen[index].time *
                                            ConfigParams::GetInstance().lidar_config.lidar_point_time_scale);
            cloud_ptr->at(index) = point;
        });
        cloud_ptr->header = cloud_leishen.header;
        cloud_ptr->is_dense = true;
        return cloud_ptr;

    } else if (LidarModel::Instance()->lidar_type == LidarType::ROBOSENSE) {
        pcl::PointCloud<VelodynePointXYZIRT> cloud_robosen;
        pcl::fromROSMsg(*msg, cloud_robosen);
        if (!cloud_robosen.is_dense) {
            // 非稠密的点云
            lidar_utils::RemoveNanFromPointCloud(cloud_robosen, cloud_robosen);
        }
        const size_t point_size = cloud_robosen.size();
        PCLCloudXYZIRT::Ptr cloud_ptr(new PCLCloudXYZIRT);
        cloud_ptr->resize(point_size);
        // 创建indics是为了在for_each中遍历cloud_velodyne
        std::vector<size_t> indics(point_size);
        // 0 ~ point_size;
        std::iota(indics.begin(), indics.end(), 0);
        std::for_each(std::execution::par, indics.begin(), indics.end(), [&](const size_t& index) {
            // 创建一个点
            PointXYZIRT point;
            point.x = cloud_robosen[index].x;
            point.y = cloud_robosen[index].y;
            point.z = cloud_robosen[index].z;
            point.intensity = cloud_robosen[index].intensity;
            point.ring = static_cast<uint8_t>(cloud_robosen[index].ring);
            // us->s
            point.time = static_cast<float>(cloud_robosen[index].time *
                                            ConfigParams::GetInstance().lidar_config.lidar_point_time_scale);
            cloud_ptr->at(index) = point;
        });
        cloud_ptr->header = cloud_robosen.header;
        cloud_ptr->is_dense = true;
        return cloud_ptr;

    } else if (LidarModel::Instance()->lidar_type == LidarType::MID360) {
        pcl::PointCloud<VelodynePointXYZIRT> cloud_mid360;
        pcl::fromROSMsg(*msg, cloud_mid360);
        if (!cloud_mid360.is_dense) {
            // 非稠密的点云
            lidar_utils::RemoveNanFromPointCloud(cloud_mid360, cloud_mid360);
        }
        const size_t point_size = cloud_mid360.size();
        PCLCloudXYZIRT::Ptr cloud_ptr(new PCLCloudXYZIRT);
        cloud_ptr->resize(point_size);
        // 创建indics是为了在for_each中遍历cloud_velodyne
        std::vector<size_t> indics(point_size);
        // 0 ~ point_size;
        std::iota(indics.begin(), indics.end(), 0);
        std::for_each(std::execution::par, indics.begin(), indics.end(), [&](const size_t& index) {
            // 创建一个点
            PointXYZIRT point;
            point.x = cloud_mid360[index].x;
            point.y = cloud_mid360[index].y;
            point.z = cloud_mid360[index].z;
            point.intensity = cloud_mid360[index].intensity;
            point.ring = 0;
            // us->s
            point.time = static_cast<float>((cloud_mid360[index].time - cloud_mid360[0].time) *
                                            ConfigParams::GetInstance().lidar_config.lidar_point_time_scale);
            cloud_ptr->at(index) = point;
        });
        cloud_ptr->header = cloud_mid360.header;
        cloud_ptr->is_dense = true;
        return cloud_ptr;
    } else if (LidarModel::Instance()->lidar_type == LidarType::AVIA) {
        pcl::PointCloud<VelodynePointXYZIRT> cloud_avia;
        pcl::fromROSMsg(*msg, cloud_avia);
        if (!cloud_avia.is_dense) {
            // 非稠密的点云
            lidar_utils::RemoveNanFromPointCloud(cloud_avia, cloud_avia);
        }
        const size_t point_size = cloud_avia.size();
        PCLCloudXYZIRT::Ptr cloud_ptr(new PCLCloudXYZIRT);
        cloud_ptr->resize(point_size);
        // 创建indics是为了在for_each中遍历cloud_velodyne
        std::vector<size_t> indics(point_size);
        // 0 ~ point_size;
        std::iota(indics.begin(), indics.end(), 0);
        std::for_each(std::execution::par, indics.begin(), indics.end(), [&](const size_t& index) {
            // 创建一个点
            PointXYZIRT point;
            point.x = cloud_avia[index].x;
            point.y = cloud_avia[index].y;
            point.z = cloud_avia[index].z;
            point.intensity = cloud_avia[index].intensity;
            // us->s
            point.time = static_cast<float>(cloud_avia[index].time *
                                            ConfigParams::GetInstance().lidar_config.lidar_point_time_scale);
            cloud_ptr->at(index) = point;
        });
        cloud_ptr->header = cloud_avia.header;
        cloud_ptr->is_dense = true;
        return cloud_ptr;

    } else {
        return nullptr;
    }
    // return
}
// velody雷达会出现的问题，需要自己计算每个点的时间偏移
// 根据第一个点，雷达的旋转速度，计算点的时间偏移
void PreProcessing::ComputePointOffsetTime(const PCLCloudXYZIRT::Ptr& cloud, double lidar_rata) {
    // 雷达线束
    const int lidar_scan_num = LidarModel::Instance()->v_scan_num_;
    // const auto point_size = cloud->size();
    const double lidar_omega = 2 * M_PI * lidar_rata;  // 雷达的角速度
    std::vector<bool> is_fisrt(lidar_scan_num, true);
    std::vector<double> yaw_fisrt_scan(lidar_scan_num, 0.0);
    std::vector<double> time_last(lidar_scan_num, 0.0);

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        const int ring = cloud->points[i].ring;
        if (ring >= lidar_scan_num) {
            // 错误的ring值
            continue;
        }
        const double yaw = std::atan2(cloud->points[i].y, cloud->points[i].x);
        // 记录每条线上遍历到第一个点的角度值
        if (is_fisrt[ring]) {
            is_fisrt[ring] = false;
            yaw_fisrt_scan[ring] = yaw;
            time_last[ring] = 0.0;
            continue;
        }
        // 根据第二个点的角度值和第一个角度值的差来计算偏移
        if (yaw <= yaw_fisrt_scan[ring]) {
            // 在第一个点的左边
            cloud->points[i].time = static_cast<float>((yaw_fisrt_scan[ring] - yaw) / lidar_omega);
        } else {
            // 在第一个点的右边
            cloud->points[i].time = static_cast<float>((yaw_fisrt_scan[ring] - yaw + 2.0 * M_PI) / lidar_omega);
        }
        // 处理小于0的时间戳
        if (cloud->points[i].time < time_last[ring]) {
            cloud->points[i].time += static_cast<float>(2.0 * M_PI / lidar_omega);
        }
        time_last[ring] = cloud->points[i].time;
    }
}

std::pair<float, float> PreProcessing::GetLidarPointMinMaxOffsetTime(const PCLCloudXYZIRT::Ptr& cloud) {
    float min = cloud->points[0].time;
    float max = cloud->points[0].time;
    for (const auto point : cloud->points) {
        if (point.time < min) {
            min = point.time;
        }
        if (point.time > max) {
            max = point.time;
        }
    }
    return {min, max};
}
}  // namespace lio