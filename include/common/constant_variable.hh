#pragma once
#include <string>

// 数据存放的目录
static const std::string kDataPath = std::string(PROJECT_SOURCE_DIR) + "/data/";
// 分割后的地图
static const std::string kTileMapFolder = std::string(PROJECT_SOURCE_DIR) + "/data/tile_map/";
//
static const std::string kTileMapIndicesFileName = "tile_map_indices.txt";
static const std::string kGlobalMapFileName = "global_map.pcd";

// 松耦合
static const std::string kFusionLoopCoupling = "";
// 滤波方式的紧耦合
static const std::string kFusionTightCouplingKF = "";
// 优化方式的紧耦合
static const std::string kFusionTightCouplingOptimization = "";
