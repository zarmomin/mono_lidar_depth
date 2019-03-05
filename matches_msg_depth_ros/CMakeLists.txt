set(MRT_PKG_VERSION 2.2)
# Modify only if you know what you are doing!
cmake_minimum_required(VERSION 2.8.12)
project(matches_msg_depth_ros)

###################
## find packages ##
###################
find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

add_executable(${PROJECT_NAME}-test test/test_matches_msg_depth_ros.cpp)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} gtest_main)

cs_install()

cs_export()
