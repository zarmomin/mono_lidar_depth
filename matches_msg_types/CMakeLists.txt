# Modify only if you know what you are doing!
cmake_minimum_required(VERSION 2.8.12)
project(matches_msg_types)
find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

#cs_add_library(${PROJECT_NAME} include/matches_msg_types/feature_point_depth.hpp
#include/matches_msg_types/feature_point.hpp
#include/matches_msg_types/tracklet.hpp
#include/matches_msg_types/tracklets.hpp)

add_executable(${PROJECT_NAME}-test test/matches_msg_types.cpp)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} gtest_main)

cs_install()

cs_export()
