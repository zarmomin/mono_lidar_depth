# Modify only if you know what you are doing!
cmake_minimum_required(VERSION 2.8.12)
project(monolidar_fusion)

###################
## find packages ##
###################
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -march=native")
add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY)
find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)

FIND_PACKAGE(PkgConfig REQUIRED)
find_package(yaml_cpp_catkin QUIET)
if(${yaml_cpp_catkin_FOUND})
  message(STATUS "Found yaml_cpp_catkin, using instead of system library.")
  set(YamlCpp_LIBRARIES ${yaml_cpp_catkin_LIBRARIES})
  set(YamlCpp_INCLUDE_DIRS ${yaml_cpp_catkin_INCLUDE_DIRS})
  catkin_package(
    INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
      eigen_catkin
      pcl_catkin
      ceres_catkin
      yaml_cpp_catkin
    DEPENDS OpenCV
  )
else()
  message(STATUS "No yaml_cpp_catkin, using yaml-cpp system library instead.")
  pkg_check_modules(YamlCpp REQUIRED yaml-cpp>=0.5)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
      eigen_catkin
      pcl_catkin
      ceres_catkin
    DEPENDS OpenCV YamlCpp
  )
endif()

catkin_simple(ALL_DEPS_REQUIRED)



add_library(${PROJECT_NAME}
src/DepthCalculationStatistics.cpp
src/DepthEstimator.cpp
src/DepthEstimatorParameters.cpp
src/GeometryHelper.cpp
src/HelperLidarRowSegmentation.cpp
src/Histogram.cpp
src/HistogramPointDepth.cpp
src/LinePlaneIntersectionBase.cpp
src/LinePlaneIntersectionCeckXZTreshold.cpp
src/LinePlaneIntersectionNormal.cpp
src/LinePlaneIntersectionOrthogonalTreshold.cpp
src/Logger.cpp
src/NeighborFinderBase.cpp
src/NeighborFinderPixel.cpp
src/PCA.cpp
src/PlaneEstimationCalcMaxSpanningTriangle.cpp
src/PlaneEstimationCheckPlanar.cpp
src/PlaneEstimationLeastSquares.cpp
src/PlaneEstimationMEstimator.cpp
src/RansacPlane.cpp
src/RoadDepthEstimatorBase.cpp
src/RoadDepthEstimatorLeastSquares.cpp
src/RoadDepthEstimatorMaxSpanningTriangle.cpp
src/RoadDepthEstimatorMEstimator.cpp
src/TresholdDepthGlobal.cpp
src/TresholdDepthLocal.cpp)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${YamlCpp_LIBRARIES})

cs_add_executable(${PROJECT_NAME}-test test/test_monolidar_fusion.cpp)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} gtest_main)

#############
## Install ##
#############

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
