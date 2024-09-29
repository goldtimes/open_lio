#pragma once

#include <livox_ros_driver/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <memory>
#include <mutex>

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

   public:
    std::shared_ptr<ros::NodeHandle> nh_;
    // 系统初始化
    std::atomic<bool> system_has_init_;

   private:
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
    ros::Publisher planner_cloud_pub;

    // server
    ros::ServiceServer map_saver_server_;
    // tf
    tf::TransformBroadcaster tf_broadcaster_;
    std::shared_ptr<ros::NodeHandle> ros_nh_;
    SLAM_MODE slam_mode_ = SLAM_MODE::UNKNOW;
};
}  // namespace lio