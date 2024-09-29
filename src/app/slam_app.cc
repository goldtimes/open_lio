#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <memory>
#include "system/SystemManager.hh"

void SignalHandler(const char* data, int size) {
    // 保存错误到文件
    std::string log_file = "/home/slamer/log/slam_error.log";
    std::ofstream fs(log_file, std::ios::app);
    std::string str(data, size);
    fs << str;
    fs.close();
    LOG(INFO) << "program exit:" << str;
}

int main(int argc, char** argv) {
    // log
    FLAGS_logtostderr = true;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::InstallFailureWriter(&SignalHandler);
    // PROJECT_SOURCE_DIR 直接用就行
    LOG(INFO) << "PROJECT_SOURCE_DIR:" << PROJECT_SOURCE_DIR;
    ros::init(argc, argv, "lio_slam_node");
    const auto nh = std::make_shared<ros::NodeHandle>();
    ros::Rate rate(1000);
    lio::SystemManager manager(nh);
    while (ros::ok()) {
        ros::spinOnce();
        // manager.Run();
        rate.sleep();
    }

    google::ShutdownGoogleLogging();

    return 0;
}