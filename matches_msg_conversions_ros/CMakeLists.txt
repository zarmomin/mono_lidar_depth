# Modify only if you know what you are doing!
cmake_minimum_required(VERSION 2.8.12)
project(matches_msg_conversions_ros)
set(CMAKE_CXX_STANDARD 14)
find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

add_library(${PROJECT_NAME}
src/convert.cpp
src/convert_opencv3.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-test test/test_matches_msg_conversions_ros.cpp)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} gtest_main)

cs_install()

cs_export()
