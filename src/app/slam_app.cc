#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <fstream>
#include <iostream>

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

    ros::init(argc, argv, "");
    ros::NodeHandle nh;

    return 0;
}