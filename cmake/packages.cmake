# 引入该目录下的.cmake文件
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
# 引用ros生成的msg header
include_directories(${CMAKE_SOURCE_DIR}/../devel/include) 
#system config
message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
# 识别有多个核心的cpu, 其实现在的cpu基本都有多核心
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

#omp
find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

# eigen 3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# sophus
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/sophus)

# glog
find_package(Glog REQUIRED)
include_directories(${Glog_INCLUDE_DIRS})

# csparse
# find_package(CSparse REQUIRED)
# include_directories(${CSPARSE_INCLUDE_DIR})

# cholmod
# find_package(Cholmod REQUIRED)
# include_directories(${CHOLMOD_INCLUDE_DIRS})

# pcl
# find_package(PCL REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${CERES_LIBRARY_DIRS})

#ceres
find_package(Ceres REQUIRED)
include_directories( ${CERES_INCLUDE_DIRS})
link_directories(${CERES_LIBRARY_DIRS})

# yaml-cpp
find_package(yaml-cpp REQUIRED)
include_directories(${yaml-cpp_INCLUDE_DIRS})

# 其他thirdparty下的内容
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/)

set(third_party_libs
        ${catkin_LIBRARIES}
        # ${g2o_libs}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${CERES_LIBRARIES}
        glog gflags
        ${yaml-cpp_LIBRARIES}
        yaml-cpp
        # robin_map
)