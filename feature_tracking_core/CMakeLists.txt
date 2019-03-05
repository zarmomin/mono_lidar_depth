# Modify only if you know what you are doing!
cmake_minimum_required(VERSION 2.8.12)
project(feature_tracking_core)
###################
## find packages ##
###################
set(CMAKE_CXX_STANDARD 14)
find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


add_library(${PROJECT_NAME} src/stereo_tracker_libviso.cpp
src/stereo_tracker.cpp
src/tracker_libviso.cpp
src/tracklet.cpp
src/utilities.cpp
src/visualization.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

catkin_add_gtest(${PROJECT_NAME}-test test/test_feature_tracking.cpp)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})

cs_install()

cs_export()
