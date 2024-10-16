#pragma once

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <livox_ros_driver/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include "common/data_type.hh"
#include "common/math_utils.hh"
#include "imu/imu_data_search.hh"
#include "lidar_process/lidar_model.hh"
#include "lidar_process/pointcloud_cluster.hh"
#include "sensors/imu.hh"
#include "slam_note/save_map.h"

namespace lio {

class FrontEnd;
class PreProcessing;
class LoopClosure;
class Localization;

enum class SLAM_MODE { UNKNOW = 0, MAPPING = 1, LOCALIZATION = 2 };

class SystemManager {
   public:
    SystemManager() = default;
    explicit SystemManager(std::shared_ptr<ros::NodeHandle> nh);
    ~SystemManager();

    void Run();
    void RunMapping();
    void RunLocalization();

   private:
    // 初始化lidar model
    static void InitLidarModel();

    void InitConfigParams();
    void InitRosPublishers();
    void InitRosSubscribers();
    void InitServer();

    bool TryToInitIMU(const IMUData& imu_data, Eigen::Vector3d& init_acc);

    void LidarLivoxCallback(const livox_ros_driver::CustomMsg::ConstPtr& livox_msg);
    void LidarStandarMsgCallback(const sensor_msgs::PointCloud2::Ptr& msg);
    void IMUCallback(const sensor_msgs::Imu::Ptr& msg);
    void EncoderCallback(const nav_msgs::Odometry::Ptr& msg);
    void InitposeCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& init_pose_cov);
    bool SaveMap(slam_note::save_map::Request& req, slam_note::save_map::Response& resp);

    void InitMapMode();
    void InitLocalizationMode();

   public:
    std::shared_ptr<ros::NodeHandle> nh_;
    // 系统初始化
    std::atomic<bool> system_has_init_;

   public:
    // sub
    ros::Subscriber imu_sub;
    ros::Subscriber lidar_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber init_pose_sub;
    // pub
    ros::Publisher lidar_path_pub;
    ros::Publisher imu_path_pub;
    ros::Publisher encoder_path_pub;
    ros::Publisher lidar_odom_pub;
    ros::Publisher imu_odom_pub;
    ros::Publisher encoder_odom_pub;
    // pointcloud
    ros::Publisher curr_scan_pub;
    ros::Publisher local_map_pub;
    ros::Publisher global_map_pub;
    ros::Publisher keyframe_path_pub;
    ros::Publisher keyframe_path_cloud_pub;
    ros::Publisher curr_keyframe_pub;
    ros::Publisher corner_cloud_pub;
    ros::Publisher plannar_cloud_pub;

    std::mutex mtx_raw_cloud_queue;
    std::mutex mtx_cloud_cluster_queue_;
    std::deque<sensor_msgs::PointCloud2Ptr> raw_cloud_queue_;
    std::deque<PointCloudClusterPtr> cloud_cluster_queue_;

    // 条件变量
    std::condition_variable cv_frontend_;
    std::condition_variable cv_localization_;
    std::condition_variable cv_preprogressing_;

    // server
    ros::ServiceServer map_saver_server_;
    // tf
    tf::TransformBroadcaster tf_broadcaster_;
    std::shared_ptr<ros::NodeHandle> ros_nh_;
    SLAM_MODE slam_mode_ = SLAM_MODE::UNKNOW;

    // imu_data_searcher
    std::shared_ptr<IMUDataSearcher> imu_data_searcher_ptr_;

    // 数据预处理模块
    PreProcessing* pre_progressing_module_;
    std::thread* progressing_thread_;
    // 建图前端模块
    std::shared_ptr<FrontEnd> frontend_module_;
    // 定位模块
    Localization* localization_module_;
    std::thread* localization_thread_;
    // 回环检测模块
    std::shared_ptr<LoopClosure> loopclosure_module_;
};
}  // namespace lio