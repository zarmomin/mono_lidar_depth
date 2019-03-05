# Modify only if you know what you are doing!
cmake_minimum_required(VERSION 2.8.12)
project(matches_conversion_ros_tool)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

generate_ros_interface_files(
  cfg/AddOutlierFlag.rosif
  cfg/MarkDepthOutlier.rosif
  cfg/SemanticLabels.rosif
)

add_executable(add_outlier_flag src/add_outlier_flag/add_outlier_flag.cpp src/add_outlier_flag/add_outlier_flag_node.cpp src/add_outlier_flag/add_outlier_flag_nodelet.cpp)
# make sure configure headers are built before any node using them
add_dependencies(add_outlier_flag ${PROJECT_NAME}_gencfg) # For dynamic_reconfigure
add_dependencies(add_outlier_flag ${PROJECT_NAME}_geninterface) # For rosinterface_handler
target_link_libraries(add_outlier_flag ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(mark_depth_outlier src/mark_depth_outlier/mark_depth_outlier.cpp src/mark_depth_outlier/mark_depth_outlier_node.cpp src/mark_depth_outlier/mark_depth_outlier_nodelet.cpp)
# make sure configure headers are built before any node using them
add_dependencies(mark_depth_outlier ${PROJECT_NAME}_gencfg) # For dynamic_reconfigure
add_dependencies(mark_depth_outlier ${PROJECT_NAME}_geninterface) # For rosinterface_handler
target_link_libraries(mark_depth_outlier ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(semantic_labels src/semantic_labels/semantic_labels.cpp src/semantic_labels/semantic_labels_node.cpp src/semantic_labels/semantic_labels_nodelet.cpp)
# make sure configure headers are built before any node using them
add_dependencies(semantic_labels ${PROJECT_NAME}_gencfg) # For dynamic_reconfigure
add_dependencies(semantic_labels ${PROJECT_NAME}_geninterface) # For rosinterface_handler
target_link_libraries(semantic_labels ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

cs_install()

cs_export()
