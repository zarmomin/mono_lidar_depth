# Modify only if you know what you are doing!
cmake_minimum_required(VERSION 2.8.12)
project(tracklets_depth_ros_tool)
set(CMAKE_CXX_STANDARD 14)
find_package(catkin_simple REQUIRED)
FIND_PACKAGE(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)

find_package(yaml_cpp_catkin QUIET)
if(${yaml_cpp_catkin_FOUND})
  message(STATUS "Found yaml_cpp_catkin, using instead of system library.")
  set(YamlCpp_LIBRARIES ${yaml_cpp_catkin_LIBRARIES})
  set(YamlCpp_INCLUDE_DIRS ${yaml_cpp_catkin_INCLUDE_DIRS})
else()
  message(STATUS "No yaml_cpp_catkin, using yaml-cpp system library instead.")
  pkg_check_modules(YamlCpp REQUIRED yaml-cpp>=0.5)
endif()

catkin_simple(ALL_DEPS_REQUIRED)

add_library(${PROJECT_NAME} src/tracklet_depth_node/parameters.cpp src/tracklet_depth_node/tracklet_depth.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${YamlCpp_LIBRARIES} ${OpenCV_LIBRARIES})

add_library(tracklet_depth_node
        src/tracklet_depth_node/tracklet_depth_node.cpp
        src/tracklet_depth_node/tracklet_depth_nodelet.cpp)
target_link_libraries(tracklet_depth_node ${PROJECT_NAME})

add_executable(${PROJECT_NAME}-test test/test_tracklet_depth_ros_tool.cpp)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} gtest_main)

cs_install()

cs_export()
