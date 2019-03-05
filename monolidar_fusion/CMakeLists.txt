# Modify only if you know what you are doing!
cmake_minimum_required(VERSION 2.8.12)
project(monolidar_fusion)

###################
## find packages ##
###################
set(CMAKE_CXX_STANDARD 14)
find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


add_library(${PROJECT_NAME} src/DepthCalculationStatistics.cpp
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
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

cs_install()

cs_export()
