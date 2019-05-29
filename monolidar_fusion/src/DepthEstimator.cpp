/*
 * DepthEstimator.cpp
 *
 *  Created on: Dec 6, 2016
 *      Author: wilczynski
 */

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <exception>
#include <limits>

#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include "monolidar_fusion/HistogramPointDepth.h"
#include "monolidar_fusion/Logger.h"
#include "monolidar_fusion/NeighborFinderPixel.h"
#include "monolidar_fusion/PCA.h"
//#include "NeighborFinderKdd.h"
//#include "Converter.h"
#include "monolidar_fusion/DepthEstimator.h"
#include "monolidar_fusion/common.h"

#include "monolidar_fusion/LinePlaneIntersectionNormal.h"
#include "monolidar_fusion/LinePlaneIntersectionOrthogonalTreshold.h"
#include "monolidar_fusion/RoadDepthEstimatorLeastSquares.h"
#include "monolidar_fusion/RoadDepthEstimatorMEstimator.h"
#include "monolidar_fusion/RoadDepthEstimatorMaxSpanningTriangle.h"

#include <chrono>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace Mono_Lidar {

bool DepthEstimator::Initialize(const Eigen::Matrix3d& K) {
  if (!_isInitializedConfig) {
    std::cout << "Call 'InitConfig' before calling 'Initialize'.";
  }
  _camera = std::make_shared<CameraPinhole>(
      _imgWitdh, _imgHeight, static_cast<float>(K(0, 0)),
      static_cast<float>(K(1, 1)), static_cast<float>(K(0, 2)),
      static_cast<float>(K(1, 2)));
  return true;
}

bool DepthEstimator::Initialize(const Eigen::Matrix3d& K,
                                const Eigen::VectorXd& d) {
  if (!_isInitializedConfig) {
    std::cout << "Call 'InitConfig' before calling 'Initialize'.";
  }

  cv::Mat_<float> d_(d.rows(), 1);
  for (int i = 0; i < d.rows(); i++) {
    d_.at<float>(i) = static_cast<float>(d(i));
  }
  _camera = std::make_shared<CameraPinhole>(
      _imgWitdh, _imgHeight, static_cast<float>(K(0, 0)),
      static_cast<float>(K(1, 1)), static_cast<float>(K(0, 2)),
      static_cast<float>(K(1, 2)), d_);
  return true;
}

bool DepthEstimator::Initialize(const std::shared_ptr<CameraPinhole>& camera) {
  if (!_isInitializedConfig) {
    std::cout << "Call 'InitConfig' before calling 'Initialize'.";
  }
  _camera = camera;
  return true;
}

bool DepthEstimator::InitializeParameters() {
  // Initialize nearest neighbor search module
  if (_parameters->neighbor_search_mode == 0) {
    // neighbor pixel search
    this->_neighborFinder = std::make_shared<NeighborFinderPixel>(
        _imgWitdh, _imgHeight, _parameters->pixelarea_search_witdh,
        _parameters->pixelarea_search_height);
    this->_neighborFinder->Initialize(_parameters);
  } else if (_parameters->neighbor_search_mode == 1) {
    //		// neighbor kdd search
    //		this->_neighborFinder = std::make_shared<NeighborFinderKdd>();
    //		this->_neighborFinder->Initialize(_parameters);
  } else
    std::cout << "neighbor_search_mode has the invalid value: " +
        std::to_string(_parameters->neighbor_search_mode);
  // Initialize treshold depth module
  if (_parameters->treshold_depth_enabled) {
    auto mode = (eTresholdDepthMode)_parameters->treshold_depth_mode;
    this->_tresholdDepthGlobal = std::make_shared<TresholdDepthGlobal>(
        mode, _parameters->treshold_depth_min, _parameters->treshold_depth_max);
  } else
    this->_tresholdDepthGlobal = NULL;
  // Initialize local treshold depth module
  if (_parameters->treshold_depth_local_enabled) {
    auto mode = (eTresholdDepthMode)_parameters->treshold_depth_local_mode;
    auto toleranceType =
        (eTresholdToleranceType)_parameters->treshold_depth_local_valuetype;
    this->_tresholdDepthLocal = std::make_shared<TresholdDepthLocal>(
        mode, toleranceType, _parameters->treshold_depth_local_value);
  } else
    this->_tresholdDepthLocal = NULL;
  // Initialize line plane intersection module
  if (_parameters->viewray_plane_orthoganality_treshold > 0)
    this->_linePlaneIntersection =
        std::make_shared<LinePlaneIntersectionOrthogonalTreshold>(
            _parameters->viewray_plane_orthoganality_treshold);
  else
    this->_linePlaneIntersection =
        std::make_shared<LinePlaneIntersectionNormal>();
  // Initialize Ransac plane for road estimation
  if (_parameters->do_use_ransac_plane) {
    // Initialize Depth Estimator for points which lie on the ground plane/road
    if (_parameters->plane_estimator_use_triangle_maximation)
      this->_roadDepthEstimator =
          std::make_shared<RoadDepthEstimatorMaxSpanningTriangle>(
              _parameters->plane_estimator_z_x_min_relation);
    else if (_parameters->plane_estimator_use_leastsquares)
      this->_roadDepthEstimator =
          std::make_shared<RoadDepthEstimatorLeastSquares>();
    else if (_parameters->plane_estimator_use_mestimator)
      this->_roadDepthEstimator =
          std::make_shared<RoadDepthEstimatorMEstimator>();
    else
      std::cout << "No road depth estimator selected.";

    if (this->_tresholdDepthGlobal != NULL)
      this->_roadDepthEstimator->EnableTresholdDepthGlobal(
          this->_tresholdDepthGlobal);

    if (this->_tresholdDepthLocal != NULL)
      this->_roadDepthEstimator->EnableTresholdDepthLocal(
          this->_tresholdDepthLocal);
  } else {
    this->_roadDepthEstimator = NULL;
  }
  // Module for constructing a maximum size plane with 3 points with a given
  // pointcloud
  if (_parameters->do_use_triangle_size_maximation)
    _planeCalcMaxSpanning =
        std::make_shared<PlaneEstimationCalcMaxSpanningTriangle>(
            _parameters->do_publish_points);
  else
    _planeCalcMaxSpanning = NULL;
  // Plane planarity checker if the plane is constructed by using exactly 3
  // points
  if (_parameters->do_check_triangleplanar_condition) {
    _checkPlanarTriangle = std::make_shared<PlaneEstimationCheckPlanar>(
        _parameters->triangleplanar_crossnorm_treshold);
  } else
    this->_checkPlanarTriangle = NULL;

  Logger::Instance().setEnabled(_parameters->do_logging);

  // camera stuff
  _imgWitdh = _parameters->image_width;
  _imgHeight = _parameters->image_height;

  _isInitialized = true;
  return true;
}

bool DepthEstimator::InitConfig(const std::string& filePath,
                                const bool printparams) {
  _parameters = std::make_shared<DepthEstimatorParameters>();
  _parameters->fromFile(filePath);

  if (printparams) _parameters->print();
  InitializeParameters();
  _isInitializedConfig = true;

  return true;
}

bool DepthEstimator::InitConfig(const bool printparams) {
  _parameters = std::make_shared<DepthEstimatorParameters>();

  if (printparams) _parameters->print();

  _isInitializedConfig = true;

  return true;
}

void DepthEstimator::setInputCloud(const Cloud::ConstPtr& cloud,
                                   GroundPlane::Ptr& groundPlane) {
  using namespace std;

  Logger::Instance().Log(Logger::MethodStart, "DepthEstimator::setInputCloud");

  // Precheck
  if (!_isInitialized) std::cout << "call of 'setInputCloud' without 'initialize'";
  _isInitializedPointCloud = true;

  _points_interpolated.clear();
  _points_interpolated_plane.clear();
  _points_triangle_corners.clear();
  _points_neighbors.clear();
  _points_groundplane.clear();
  _points.Clear();

  // Change coordinate frame
  Transform_Cloud_LidarToCamera(cloud);
  try{
    if (_parameters->neighbor_search_mode == 0) {
      // use next pixel neighbor search
      // Transform the visible lidar points if neighbor search mode is pixel based
      auto neighborFinder =
          std::dynamic_pointer_cast<NeighborFinderPixel>(_neighborFinder);

      neighborFinder->InitializeLidarProjection(this->_points._points_cs_image, this->_points._points_cs_camera);
    } else if (_parameters->neighbor_search_mode == 1) {
      // use kdd tree
      //		auto neighborFinder =
      // std::dynamic_pointer_cast<NeighborFinderKdd>(_neighborFinder);
      //		neighborFinder->InitKdTree(this->_points._points_cs_image_visible);
    } else {
      std::cout << "neighbor_search_mode has the invalid value: " +
          std::to_string(_parameters->neighbor_search_mode);
    }
  } catch (...)
  {
    std::cout << "\nError here.\n";
  }
  Logger::Instance().Log(Logger::MethodEnd, "DepthEstimator::setInputCloud");
}

void DepthEstimator::setInputCloud(const Cloud::ConstPtr& cloud) {
  using namespace std;

  Logger::Instance().Log(Logger::MethodStart, "DepthEstimator::setInputCloud");

  // Precheck
  if (!_isInitialized) std::cout << "call of 'setInputCloud' without 'initialize'";
  _isInitializedPointCloud = true;

  _points_interpolated.clear();
  _points_interpolated_plane.clear();
  _points_triangle_corners.clear();
  _points_neighbors.clear();
  _points_groundplane.clear();
  _points.Clear();

  Transform_Cloud_LidarToCamera(cloud);

  if (_parameters->neighbor_search_mode == 0) {
    // use next pixel neighbor search
    // Transform the visible lidar points if neighbor search mode is pixel based
    auto neighborFinder =
        std::dynamic_pointer_cast<NeighborFinderPixel>(_neighborFinder);

    neighborFinder->InitializeLidarProjection(this->_points._points_cs_image, this->_points._points_cs_camera);
  } else if (_parameters->neighbor_search_mode == 1) {
    // use kdd tree
    //		auto neighborFinder =
    // std::dynamic_pointer_cast<NeighborFinderKdd>(_neighborFinder);
    //		neighborFinder->InitKdTree(this->_points._points_cs_image_visible);
  } else {
    std::cout << "neighbor_search_mode has the invalid value: " +
        std::to_string(_parameters->neighbor_search_mode);
  }

  Logger::Instance().Log(Logger::MethodEnd, "DepthEstimator::setInputCloud");
}

void DepthEstimator::setInputCloud(const Cloud::ConstPtr& cloud,
                                   const Eigen::Vector4d& qCL_WXYZ,
                                   const Eigen::Vector3d& rCL) {
  using namespace std;

  Logger::Instance().Log(Logger::MethodStart, "DepthEstimator::setInputCloud");

  // Precheck
  if (!_isInitialized) std::cout << "call of 'setInputCloud' without 'initialize'";
  _isInitializedPointCloud = true;

  _points_interpolated.clear();
  _points_interpolated_plane.clear();
  _points_triangle_corners.clear();
  _points_neighbors.clear();
  _points_groundplane.clear();
  _points.Clear();

  // Change coordinate frame
  cv::Mat_<float> R_CL;
  quaternionToRotationMatrix(qCL_WXYZ, R_CL);
  cv::Mat_<float> r_CL(3,1);

  r_CL << static_cast<float>(rCL[0]), static_cast<float>(rCL[1]),
      static_cast<float>(rCL[2]);

  Transform_Cloud_LidarToCamera(cloud, R_CL, r_CL);

  if (_parameters->neighbor_search_mode == 0) {
    // use next pixel neighbor search
    // Transform the visible lidar points if neighbor search mode is pixel based
    auto neighborFinder =
        std::dynamic_pointer_cast<NeighborFinderPixel>(_neighborFinder);

    neighborFinder->InitializeLidarProjection(this->_points._points_cs_image, this->_points._points_cs_camera);
  } else if (_parameters->neighbor_search_mode == 1) {
    // use kdd tree
    //		auto neighborFinder =
    // std::dynamic_pointer_cast<NeighborFinderKdd>(_neighborFinder);
    //		neighborFinder->InitKdTree(this->_points._points_cs_image_visible);
  } else {
    std::cout << "neighbor_search_mode has the invalid value: " +
        std::to_string(_parameters->neighbor_search_mode);
  }

  Logger::Instance().Log(Logger::MethodEnd, "DepthEstimator::setInputCloud");
}

void DepthEstimator::getCloudCameraCs(Cloud::Ptr& pointCloud_cam_cs) {
  pointCloud_cam_cs->clear();

  for (auto pt : _points._points_cs_camera) {
    pcl::PointXYZI point;
    point.x = pt.x;
    point.y = pt.y;
    point.z = pt.z;
    point.intensity = 1;

    pointCloud_cam_cs->points.push_back(point);
  }

  pointCloud_cam_cs->width = (uint32_t)pointCloud_cam_cs->points.size();
  pointCloud_cam_cs->height = 1;
  pointCloud_cam_cs->is_dense = false;
}

void DepthEstimator::getCloudNeighbors(std::vector<cv::Point2f>& pts) {
  pts = _points_neighbors;
}

void DepthEstimator::getCloudInterpolated(Cloud::Ptr& pointCloud_interpolated) {
  FillCloud(_points_interpolated, pointCloud_interpolated);
}

void DepthEstimator::getCloudInterpolatedPlane(
    Cloud::Ptr& pointCloud_interpolated_plane) {
  FillCloud(_points_interpolated, pointCloud_interpolated_plane);
}

void DepthEstimator::getCloudTriangleCorners(
    Cloud::Ptr& pointCloud_triangle_corner) {
  FillCloud(_points_triangle_corners, pointCloud_triangle_corner);
}

void DepthEstimator::FillCloud(const std::vector<Eigen::Vector3d> content,
                               const Cloud::Ptr& cloud) {
  int count = content.size();
  cloud->clear();

  for (int i = 0; i < count; i++) {
    double x = content[i].x();
    double y = content[i].y();
    double z = content[i].z();
    pcl::PointXYZI point;
    point.x = x;
    point.y = y;
    point.z = z;
    point.intensity = 1;

    cloud->points.push_back(point);
  }

  cloud->width = (uint32_t)cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = false;
}

void DepthEstimator::CutPointCloud(const Cloud::ConstPtr& cloud_in,
                                   const Cloud::Ptr& cloud_out) {
  pcl::PointCloud<Point>::Ptr cloud_filtered(new pcl::PointCloud<Point>);

  pcl::PassThrough<Point> pass;
  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(0, std::numeric_limits<float>::max());
  pass.filter(*cloud_filtered);

  cloud_out->header.frame_id = cloud_filtered->header.frame_id;
  cloud_out->header.stamp = cloud_filtered->header.stamp;

  for (const auto& pt : cloud_filtered->points) {
    // nico todo: this implicitly assumes a camera FOV of 90deg & a camera
    // parallel to the laser
    if (fabs(pt.x) > fabs(pt.y)) {
      pcl::PointXYZI newPoint;
      newPoint.x = pt.x;
      newPoint.y = pt.y;
      newPoint.z = pt.z;
      newPoint.intensity = pt.intensity;

      cloud_out->points.push_back(newPoint);
    }
  }
}

void DepthEstimator::getPointsCloudImageCs(
    std::vector<cv::Point2f>& visiblePointsImageCs,
    std::vector<double>& depths) {
  visiblePointsImageCs = this->_points._points_cs_image;
  for (int i = 0; i < visiblePointsImageCs.size(); i++) {
    depths.push_back(getPointDepthCamVisible(i));
  }
}

void DepthEstimator::getCloudRansacPlane(Cloud::Ptr& pointCloud_plane_ransac) {
  FillCloud(_points_groundplane, pointCloud_plane_ransac);
}

const DepthCalculationStatistics& DepthEstimator::getDepthCalcStats() {
  return this->_depthCalcStats;
}

void DepthEstimator::CalculateDepth(const Cloud::ConstPtr& pointCloud,
                                    const Eigen::Matrix2Xd& points_image_cs,
                                    Eigen::VectorXd& points_depths,
                                    GroundPlane::Ptr& ransacPlane) {
  setInputCloud(pointCloud, ransacPlane);
  CalculateDepth(points_image_cs, points_depths, ransacPlane);
}

void DepthEstimator::CalculateDepth(const Cloud::ConstPtr& pointCloud,
                                    const Eigen::Matrix2Xd& points_image_cs,
                                    Eigen::VectorXd& points_depths) {
  GroundPlane::Ptr tmp = nullptr;
  setInputCloud(pointCloud, tmp);
  Eigen::VectorXi depthTypes(points_image_cs.cols());
  CalculateDepth(points_image_cs, points_depths, depthTypes, tmp);
}

void DepthEstimator::CalculateDepth(const Cloud::ConstPtr& pointCloud,
                                    const Eigen::Matrix2Xd& points_image_cs,
                                    Eigen::VectorXd& points_depths,
                                    Eigen::VectorXi& resultType,
                                    GroundPlane::Ptr& ransacPlane) {
  setInputCloud(pointCloud, ransacPlane);
  CalculateDepth(points_image_cs, points_depths, resultType, ransacPlane);
}

void DepthEstimator::CalculateDepth(
    const Eigen::Matrix2Xd& featurePoints_image_cs,
    Eigen::VectorXd& points_depths, const GroundPlane::Ptr& ransacPlane) {
  Eigen::VectorXi depthTypes(featurePoints_image_cs.cols());
  CalculateDepth(featurePoints_image_cs, points_depths, depthTypes,
                 ransacPlane);
}

void DepthEstimator::CalculateDepth(
    const Eigen::Matrix2Xd& featurePoints_image_cs,
    Eigen::VectorXd& points_depths) {
  Eigen::VectorXi depthTypes(featurePoints_image_cs.cols());
  GroundPlane::Ptr tmp = nullptr;
  CalculateDepth(featurePoints_image_cs, points_depths, depthTypes, tmp);
}

void DepthEstimator::CalculateDepth(
    const Eigen::Matrix2Xd& featurePoints_image_cs,
    Eigen::VectorXd& points_depths, Eigen::VectorXi& resultType,
    const GroundPlane::Ptr& ransacPlane) {
  Mono_Lidar::Logger::Instance().Log(Mono_Lidar::Logger::MethodStart,
                                     "DepthEstimator::CalculateDepth Start: ");

  using namespace std;

  // Precheck
  if (!_isInitializedPointCloud || _parameters->set_all_depths_to_zero) {
    std::cout << "\nDepthEstimator: need to set pointcloud first";
    resultType.setConstant(1);
    points_depths.setConstant(-1);

    return;
  }

  int imgPointCount = featurePoints_image_cs.cols();

  if (_parameters->do_depth_calc_statistics) _depthCalcStats.Clear();


  //#pragma omp parallel for
  for (int i = 0; i < imgPointCount; i++) {
    //double imgPointX = featurePoints_image_cs(0, i);
    //double imgPointY = featurePoints_image_cs(1, i);
    const Eigen::Vector2d& imgPoint = featurePoints_image_cs.col(i);
    if (imgPoint.squaredNorm() > 0)
    {
      std::shared_ptr<DepthCalcStatsSinglePoint> stats = NULL;

      if (_parameters->do_debug_singleFeatures)
        stats = std::make_shared<DepthCalcStatsSinglePoint>();
      auto depthPair = this->CalculateDepth(imgPoint, ransacPlane, stats);
      points_depths(i) = depthPair.second;
      resultType(i) = depthPair.first;

      if (_parameters->do_debug_singleFeatures &&
          (depthPair.first != DepthResultType::Success || depthPair.second < 0)) {
        std::cout << "\n"
                  << i << "th feature w/ result "
                  << DepthResultTypeStrings[depthPair.first] << "\n";
      }
    }
    else{
      points_depths(i) = -1;
      resultType(i) = DepthResultType::Unspecified;
    }
    /*
    for (auto pt : stats->_neighbors3d)
    {
      std::cout << "\nneigbor " << std::get<0>(pt) << ", " << std::get<1>(pt) <<
    ", "<< std::get<2>(pt);
    }*/
  }

  if (_parameters->do_depth_calc_statistics) {
    int featuresCount = featurePoints_image_cs.cols();
    _depthCalcStats.SetPointCount(featuresCount);
  }

  Mono_Lidar::Logger::Instance().Log(Mono_Lidar::Logger::MethodEnd,
                                     "DepthEstimator::CalculateDepth.");
}

std::pair<DepthResultType, double> DepthEstimator::CalculateDepth(
    const Eigen::Vector2d& featurePoint_image_cs,
    const GroundPlane::Ptr& ransacPlane,
    std::shared_ptr<DepthCalcStatsSinglePoint> calcStats) {
  using namespace std;

  // Debug log feature
  if (calcStats != nullptr) {
    calcStats->_featureX = featurePoint_image_cs.x();
    calcStats->_featureY = featurePoint_image_cs.y();
  }

  // Get the neighbor pixels around the given feature point
  std::vector<uint16_t> neighborIndicesCut;
  std::vector<Eigen::Vector3d> neighbors;
  auto result =
      std::make_pair<DepthResultType, double>(DepthResultType::Unspecified, -1);

  // Get the neighbors around the feature point (on the image plane) as 3D lidar
  // points
  if (!this->CalculateNeighbors(featurePoint_image_cs, neighborIndicesCut,
                                neighbors, calcStats))
    return std::pair<DepthResultType, double>(
        DepthResultType::RadiusSearchInsufficientPoints, -1);

  // Segment the neighbors due to depth using a histogram
  std::vector<Eigen::Vector3d> neighborsSegmented;
  std::vector<int> neighborsSegmentedIndex;

  if (!this->CalculateDepthSegmentation(neighbors, neighborIndicesCut,
                                        neighborsSegmented,
                                        neighborsSegmentedIndex, calcStats)) {
    result = std::pair<DepthResultType, double>(
        DepthResultType::HistogramNoLocalMax, -1);
  }
  // Calculate depth
  bool checkPlanar = _checkPlanarTriangle != NULL;
  if (result.first != DepthResultType::HistogramNoLocalMax) {
    result = this->CalculateDepthSegmented(
        featurePoint_image_cs, neighborsSegmented, calcStats, checkPlanar);

    if (result.first == DepthResultType::Success) return result;
  }

  // Special treatment for road features
  DepthResultType resultOld = result.first;
  if ((ransacPlane != nullptr) && (this->_roadDepthEstimator != NULL)) {
    // calculate neighbors in a wider area for the road
    neighborIndicesCut.clear();
    neighbors.clear();

    if (!this->CalculateNeighbors(featurePoint_image_cs, neighborIndicesCut,
                                  neighbors, calcStats, 2.0, 1.5))
      return std::pair<DepthResultType, double>(
          DepthResultType::RadiusSearchInsufficientPoints, -1);

    // get the neighbor points which are inliers of the plane estimation
    if (!this->CalculateDepthSegmentationPlane(neighbors, neighborIndicesCut,
                                               neighborsSegmented, calcStats,
                                               ransacPlane))
      return std::pair<DepthResultType, double>(resultOld, -1);

    // calculate the depth with points which lay on the ground plane
    Eigen::Vector3d intersectionPoint;
    result = this->_roadDepthEstimator->CalculateDepth(
        featurePoint_image_cs, _camera, neighborsSegmented, intersectionPoint);
  }

  return result;
}

int DepthEstimator::CalcDepthSegmentionRegionGrowing(
    const Eigen::Vector2d& featurePoint_image_cs,
    std::vector<Eigen::Vector3d>& neighborsSegmented,
    std::vector<int>& imgNeighborsSegmentedIndex) {
  return 0;
}

bool DepthEstimator::CalculateNeighbors(
    const Eigen::Vector2d& featurePoint_image_cs,
    std::vector<uint16_t>& neighborIndicesCut,
    std::vector<Eigen::Vector3d>& neighbors,
    std::shared_ptr<DepthCalcStatsSinglePoint> calcStats, const float scaleX,
    const float scaleY) {
  using namespace std;

  // get the indices of the neighbors in the image
  this->_neighborFinder->getNeighbors(
      featurePoint_image_cs, this->_points._points_cs_camera,
      neighborIndicesCut, calcStats, scaleX, scaleY);

  // todo: remove when debugging is done
  // if (_parameters->do_logging) {
  for (int i = 0; i < neighborIndicesCut.size(); i++) {
    cv::Point pt = this->_points._points_cs_image[neighborIndicesCut[i]];
    this->_points_neighbors.push_back(
        this->_points._points_cs_image[neighborIndicesCut[i]]);
  }
  //}

  // get the 3D neighbor points from the pointcloud using the given indices
  this->_neighborFinder->getNeighbors(this->_points._points_cs_camera,
                                      neighborIndicesCut, neighbors);

  return neighbors.size() >= (uint)_parameters->radiusSearch_count_min;
}

bool DepthEstimator::CalculateNearestPoint(
    const std::vector<Eigen::Vector3d>& neighbors,
    const std::vector<int>& neighborsIndexCut, Eigen::Vector3d& nearestPoint,
    int& nearestPointIndex) {
  // Create depth vector of the 3d lidar points
  Eigen::VectorXd points3dDepth(neighbors.size());

  for (uint i = 0; i < neighbors.size(); i++) {
    double distance = neighbors[i].z();
    points3dDepth[i] = distance;
  }

  float minDepth = std::numeric_limits<double>::max();
  int minIndex = -1;
  int depthCount = points3dDepth.rows();

  for (int i = 0; i < depthCount; i++) {
    if (points3dDepth(i) < minDepth) {
      minDepth = points3dDepth(i);
      minIndex = i;
    }
  }

  // check if no point available
  if (minIndex == -1) {
    return false;
  }

  // nearest point has been found
  nearestPoint = neighbors.at(minIndex);
  nearestPointIndex = neighborsIndexCut.at(minIndex);

  return true;
}

bool DepthEstimator::CalculateDepthSegmentation(
    const std::vector<Eigen::Vector3d>& neighbors,
    const std::vector<uint16_t>& neighborsIndexCut,
    std::vector<Eigen::Vector3d>& pointsSegmented,
    std::vector<int>& pointsSegmentedIndex,
    std::shared_ptr<DepthCalcStatsSinglePoint> calcStats) {
  using namespace std;

  pointsSegmented.clear();
  pointsSegmentedIndex.clear();

  if (_parameters->do_use_histogram_segmentation) {
    // Create depth vector of the 3d lidar points
    Eigen::VectorXd points3dDepth(neighbors.size());

    for (uint i = 0; i < neighbors.size(); i++) {
      double distance = neighbors[i].norm();
      points3dDepth[i] = std::min(distance, 999.);
    }

    double lowBorder;
    double highBorder;

    bool localMaxFound = PointHistogram::FilterPointsMinDistBlob(
        neighbors, neighborsIndexCut, points3dDepth,
        _parameters->histogram_segmentation_bin_witdh,
        _parameters->histogram_segmentation_min_pointcount, pointsSegmented,
        pointsSegmentedIndex, lowBorder, highBorder, calcStats);

    if (!localMaxFound) return false;
  } else {
    pointsSegmented = neighbors;
  }

  return true;
}

bool DepthEstimator::CalculateDepthSegmentationPlane(
    const std::vector<Eigen::Vector3d>& neighbors,
    const std::vector<uint16_t> neighborIndices,
    std::vector<Eigen::Vector3d>& pointsSegmented,
    std::shared_ptr<DepthCalcStatsSinglePoint> calcStats,
    GroundPlane::Ptr ransacPlane) {
  // check precondition
  if (neighbors.size() != neighborIndices.size()) {
    std::cout <<(
        "DepthEstimator::CalculateDepthSegmentationPlane: neighbors.size() (" +
        std::to_string(neighbors.size()) + ") != neighborIndices.size() (" +
        std::to_string(neighborIndices.size()) + ")");
  }

  pointsSegmented.clear();

  if (ransacPlane == nullptr) {
    std::cout << "Ransac plane is nullptr" << std::endl;
  }
  const auto& planeCoeffs = ransacPlane->getModelCoeffs();

  double treshold = _parameters->ransac_plane_point_distance_treshold;

  for (size_t i = 0; i < neighbors.size(); i++) {
    int index = neighborIndices[i];

    Eigen::Vector3d point_lidar_cs = neighbors[i];
    pcl::PointXYZ point(point_lidar_cs.x(), point_lidar_cs.y(),
                        point_lidar_cs.z());
    double distance = pcl::pointToPlaneDistance(point, planeCoeffs);

    if (distance > treshold) return false;

    if (ransacPlane->CheckPointInPlane(index)) {
      // use treshold
      pointsSegmented.push_back(neighbors[i]);

      // #pragma omp critical
      { _points_triangle_corners.push_back(neighbors[i]); }
    }
  }

  // Check if enough points are in the set to span a plane
  if (pointsSegmented.size() < 3) {
    return false;
  }

  // Check if the z size is bigger than the x size
  double minX = std::numeric_limits<int>::max();
  double maxX = std::numeric_limits<int>::min();
  double minZ = std::numeric_limits<int>::max();
  double maxZ = std::numeric_limits<int>::min();

  for (const auto& pt : pointsSegmented) {
    if (pt.x() < minX)
      minX = pt.x();
    else if (pt.x() > maxX)
      maxX = pt.x();

    if (pt.z() < minZ)
      minZ = pt.z();
    else if (pt.z() > maxZ)
      maxZ = pt.z();
  }

  double deltaX = maxX - minX;
  double deltaZ = maxZ - minZ;

  if (deltaX >= deltaZ) {
    // return false;
  }

  // Points segmented
  Eigen::Vector3d pos1 = Eigen::Vector3d::Zero();

  for (const auto& pt : pointsSegmented) pos1 += pt;
  pos1 /= pointsSegmented.size();

  // neighbors points
  Eigen::Vector3d pos2 = Eigen::Vector3d::Zero();

  for (const auto& pt : neighbors) {
    pos2 += pt;
  }
  pos2 /= neighbors.size();

  //	Logger::Instance().PrintEigenVector(pos1, "Mean segmented");
  //	Logger::Instance().PrintEigenVector(pos2, "Mean all");
  //	std::cout << "Sizes: " << pointsSegmented.size() << "/" <<
  // neighbors.size() << std::endl;

  // calculate if the distance of the mean point is under a treshold

  //	Eigen::Vector3d point_lidar_cs = _transform_cam_to_lidar * pos1;
  //	pcl::PointXYZ point(point_lidar_cs.x(), point_lidar_cs.y(),
  // point_lidar_cs.z());
  //	double distance = pcl::pointToPlaneDistance(point, planeCoeffs);
  //
  //	if (distance > treshold)
  //	{
  //		return false;
  //	}

  //	for (const auto& pt : neighbors)
  //	{
  //		Eigen::Vector3d point_lidar_cs = _transform_cam_to_lidar * pos2;
  //		pcl::PointXYZ point(point_lidar_cs.x(), point_lidar_cs.y(),
  // point_lidar_cs.z());
  //		double distance = pcl::pointToPlaneDistance(point, planeCoeffs);
  //
  //		if (distance > treshold)
  //		{
  //			return false;
  //		}
  //	}

  return true;
}

std::pair<DepthResultType, double> DepthEstimator::CalculateDepthSegmented(
    const Eigen::Vector2d& point_image_cs,
    const std::vector<Eigen::Vector3d>& pointsSegmented,
    std::shared_ptr<DepthCalcStatsSinglePoint> calcStats,
    const bool checkPlanarTriangle) {
  using namespace std;

  // Get the spanning triangle from lidar points
  int corner1 = 0;
  int corner2 = 1;
  int corner3 = 2;

  if (!_parameters->do_use_PCA && (this->_planeCalcMaxSpanning != nullptr)) {
    if (!this->_planeCalcMaxSpanning->CalculatePlaneCorners(
            pointsSegmented, corner1, corner2, corner3))
      return std::pair<DepthResultType, double>(
          DepthResultType::TriangleNotPlanarInsufficientPoints, -1);
  } else {
    if (pointsSegmented.size() < 3)
      return std::pair<DepthResultType, double>(
          DepthResultType::HistogramNoLocalMax, -1);
  }

  if (!_parameters->do_use_PCA && (_checkPlanarTriangle != nullptr)) {
    if (!_checkPlanarTriangle->CheckPlanar(pointsSegmented[corner1],
                                           pointsSegmented[corner2],
                                           pointsSegmented[corner3])) {
      return std::pair<DepthResultType, double>(
          DepthResultType::TriangleNotPlanar, -1);
    }
  }

  // get the depth of the feature point
  Eigen::Vector3d viewingRayDirection;
  _camera->getViewingRays(point_image_cs, viewingRayDirection);

  if (viewingRayDirection.z() < 0) viewingRayDirection *= -1;

  Eigen::Vector3d intersectionPoint;
  double depth;

  if (_parameters->do_use_PCA) {
    Eigen::MatrixXd pointCloud(3, pointsSegmented.size());

    for (size_t i = 0; i < pointsSegmented.size(); i++) {
      pointCloud(0, i) = pointsSegmented[i].x();
      pointCloud(1, i) = pointsSegmented[i].y();
      pointCloud(2, i) = pointsSegmented[i].z();
    }

    // analyse the pointcloud using pca
    auto pca = Mono_LidarPipeline::PCA(_parameters, pointCloud);
    auto pcaResult = pca.getResult();

    switch (pcaResult) {
      case Mono_LidarPipeline::PCA_Result::Point:
        return std::pair<DepthResultType, double>(DepthResultType::PcaIsPoint,
                                                  -1);
      case Mono_LidarPipeline::PCA_Result::Linear:
        return std::pair<DepthResultType, double>(DepthResultType::PcaIsLine,
                                                  -1);
      case Mono_LidarPipeline::PCA_Result::Cubic:
        return std::pair<DepthResultType, double>(DepthResultType::PcaIsCubic,
                                                  -1);
      default:
        break;
    }

    // debug log PCA
    if (calcStats != NULL) {
      switch (pcaResult) {
        case Mono_LidarPipeline::PCA_Result::Point:
          calcStats->_pcaResult = "Point";
          break;
        case Mono_LidarPipeline::PCA_Result::Linear:
          calcStats->_pcaResult = "Line";
          break;
        case Mono_LidarPipeline::PCA_Result::Cubic:
          calcStats->_pcaResult = "Cubic";
          break;
        default:
          break;
      }

      calcStats->_pcaEigenVector1 = pca.getEigenVector1();
      calcStats->_pcaEigenVector2 = pca.getEigenVector2();
      calcStats->_pcaEigenVector3 = pca.getEigenVector3();
      calcStats->_pcaPlaneAnchor = pca.getPlaneAnchorPoint();
      calcStats->_pcaPlaneNormal = pca.getPlaneNormal();
    }

    // continues if the cloud is (approximately) a plane
    if (!_linePlaneIntersection->GetIntersectionPoint(
            pca.getPlaneNormal(), pca.getPlaneAnchorPoint(),
            viewingRaySupportPoint, viewingRayDirection, intersectionPoint,
            depth))
      return std::pair<DepthResultType, double>(
          DepthResultType::PlaneViewrayNotOrthogonal, -1);
  } else {
    if (!_linePlaneIntersection->GetIntersectionDistance(
            pointsSegmented[corner1], pointsSegmented[corner2],
            pointsSegmented[corner3], viewingRaySupportPoint,
            viewingRayDirection, depth))
      return std::pair<DepthResultType, double>(
          DepthResultType::PlaneViewrayNotOrthogonal, -1);
  }

  if (this->_tresholdDepthGlobal != NULL) {
    auto result = this->_tresholdDepthGlobal->CheckInDepth(depth);
    if (result == eTresholdResult::SmallerMin)
      return std::pair<DepthResultType, double>(
          DepthResultType::TresholdDepthGlobalSmallerMin, -1);
    else if (result == eTresholdResult::GreaterMax)
      return std::pair<DepthResultType, double>(
          DepthResultType::TresholdDepthGlobalGreaterMax, -1);
  }
  return std::pair<DepthResultType, double>(DepthResultType::Success, depth);
}

void DepthEstimator::LogDepthCalcStats(const DepthResultType depthResult) {
  switch (depthResult) {
    case DepthResultType::CornerBehindCamera:
      _depthCalcStats.AddCornerBehindCamera();
      break;
    case DepthResultType::HistogramNoLocalMax:
      _depthCalcStats.AddHistogramNoLocalMax();
      break;
    case DepthResultType::PcaIsCubic:
      _depthCalcStats.AddPCAIsCubic();
      break;
    case DepthResultType::PcaIsLine:
      _depthCalcStats.AddPCAIsLine();
      break;
    case DepthResultType::PcaIsPoint:
      _depthCalcStats.AddPCAIsPoint();
      break;
    case DepthResultType::PlaneViewrayNotOrthogonal:
      _depthCalcStats.AddPlaneViewrayNotOrthogonal();
      break;
    case DepthResultType::RadiusSearchInsufficientPoints:
      _depthCalcStats.AddRadiusSearchInsufficientPoints();
      break;
    case DepthResultType::Success:
      _depthCalcStats.AddSuccess();
      break;
    case DepthResultType::TresholdDepthGlobalGreaterMax:
      _depthCalcStats.AddTresholdDepthGlobalGreaterMax();
      break;
    case DepthResultType::TresholdDepthGlobalSmallerMin:
      _depthCalcStats.AddTresholdDepthGlobalSmallerMin();
      break;
    case DepthResultType::TresholdDepthLocalGreaterMax:
      _depthCalcStats.AddTresholdDepthLocalGreaterMax();
      break;
    case DepthResultType::TresholdDepthLocalSmallerMin:
      _depthCalcStats.AddTresholdDepthLocalSmallerMin();
      break;
    case DepthResultType::TriangleNotPlanar:
      _depthCalcStats.AddTriangleNotPlanar();
      break;
    case DepthResultType::TriangleNotPlanarInsufficientPoints:
      _depthCalcStats.AddTriangleNotPlanarInsufficientPoints();
      break;
    case DepthResultType::InsufficientRoadPoints:
      _depthCalcStats.AddInsufficientRoadPoints();
      break;
    default:

      break;
  }
}

void DepthEstimator::Transform_Cloud_LidarToCamera(
    const Cloud::ConstPtr& cloud_lidar_cs, const cv::Mat_<float>& R_CL,
    const cv::Mat_<float>& r_CL) {
  Logger::Instance().Log(Logger::MethodStart,
                         "DepthEstimator::Transform_Cloud_LidarToCamera");

  using namespace std;

  // todo: there is a LOT of potential for saving time & memory here
  Logger::Instance().Log(Logger::MethodStart,
                         "DepthEstimator::Transform_Cloud_LidarToCamera::Map");

  _points._points_cs_camera.reserve(cloud_lidar_cs->width);
  for (auto pt : *cloud_lidar_cs) {
    cv::Point3f p(pt.x, pt.y, pt.z);
    _points._points_cs_camera.push_back(p);
  }

  Logger::Instance().Log(Logger::MethodEnd,
                         "DepthEstimator::Transform_Cloud_LidarToCamera::Map");

  Logger::Instance().Log(
      Logger::MethodStart,
      "DepthEstimator::Transform_Cloud_LidarToCamera::CamstuffCV");

  _camera->getImagePoints(_points._points_cs_camera, _points._points_cs_image,
                          R_CL, r_CL);

  Logger::Instance().Log(
      Logger::MethodEnd,
      "DepthEstimator::Transform_Cloud_LidarToCamera::CamstuffCV");

  Logger::Instance().Log(
      Logger::MethodStart,
      "DepthEstimator::Transform_Cloud_LidarToCamera::AssignmentShit");

  bool removalTracker[_points._points_cs_image.size()];
  int i = 0;
  for (auto pt : _points._points_cs_image) {
    if (pt.x < 0 || pt.x >= _imgWitdh || pt.y < 0 || pt.y >= _imgHeight)
      removalTracker[i] = true;
    i++;
  }
  int j = -1;
  _points._points_cs_image.erase(
      std::remove_if(_points._points_cs_image.begin(),
                     _points._points_cs_image.end(),
                     [this, &j, &removalTracker](const cv::Point2f& pt) {
                       j++;
                       return removalTracker[j];
                     }),
      _points._points_cs_image.end());
  int k = -1;
  _points._points_cs_camera.erase(
      std::remove_if(_points._points_cs_camera.begin(),
                     _points._points_cs_camera.end(),
                     [&removalTracker, &k](const cv::Point3f& pt) {
                       k++;
                       return removalTracker[k];
                     }),
      _points._points_cs_camera.end());

  Logger::Instance().Log(
      Logger::MethodEnd,
      "DepthEstimator::Transform_Cloud_LidarToCamera::AssignmentShit");

  // Debug info
  Logger::Instance().Log(Logger::MethodEnd,
                         "DepthEstimator::Transform_Cloud_LidarToCamera");
}

} /* namespace lidorb_ros_tool */
