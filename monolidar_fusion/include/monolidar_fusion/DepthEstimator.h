/*
 * DepthEstimator.h
 *
 *  Created on: Dec 6, 2016
 *      Author: wilczynski
 */

#pragma once

#include <memory>
#include <Eigen/Eigen> // IWYU pragma: keep
#include <Eigen/StdVector>
//#include <opencv/cxcore.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include "monolidar_fusion/DepthCalcStatsSinglePoint.h"
#include "monolidar_fusion/DepthCalculationStatistics.h"
#include "monolidar_fusion/DepthEstimatorParameters.h"
#include "monolidar_fusion/HelperLidarRowSegmentation.h"
#include "monolidar_fusion/LinePlaneIntersectionBase.h"
#include "monolidar_fusion/NeighborFinderBase.h"
#include "monolidar_fusion/PlaneEstimationCalcMaxSpanningTriangle.h"
#include "monolidar_fusion/PlaneEstimationCheckPlanar.h"
#include "monolidar_fusion/PointcloudData.h"
#include "monolidar_fusion/TresholdDepthGlobal.h"
#include "monolidar_fusion/TresholdDepthLocal.h"
#include "monolidar_fusion/camera_pinhole.h"
#include "monolidar_fusion/eDepthResultType.h"
#include "monolidar_fusion/common.h"

namespace Mono_Lidar {

class DepthEstimator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    std::map<DepthResultType, std::string> DepthResultTypeMap{
        {Success, "Success"},
        {RadiusSearchInsufficientPoints, "RadiusSearchInsufficientPoints"},
        {HistogramNoLocalMax, "HistogramNoLocalMax"},
        {TresholdDepthGlobalGreaterMax, "TresholdDepthGlobalGreaterMax"},
        {TresholdDepthGlobalSmallerMin, "TresholdDeptGlobalhSmallerMin"},
        {TresholdDepthLocalGreaterMax, "TresholdDepthLocalGreaterMax"},
        {TresholdDepthLocalSmallerMin, "TresholdDeptLocalhSmallerMin"},
        {TriangleNotPlanar, "TriangleNotPlanar"},
        {TriangleNotPlanarInsufficientPoints, "TriangleNotPlanarInsufficientPoints"},
        {CornerBehindCamera, "CornerBehindCamera"},
        {PlaneViewrayNotOrthogonal, "PlaneViewrayNotOrthogonal"},
        {PcaIsPoint, "PcaIsPoint"},
        {PcaIsLine, "PcaIsLine"},
        {PcaIsCubic, "PcaIsCubic"},
        {InsufficientRoadPoints, "InsufficientRoadPoints"}};

    using Point = pcl::PointXYZI;
    using Cloud = pcl::PointCloud<Point>;
    using UniquePtr = std::unique_ptr<DepthEstimator>;
    using SharedPtr = std::shared_ptr<DepthEstimator>;

    /*
     * Must be called once before using.
     *
     * @param camera Intrinsic camera parameters
     * @param transform_lidar_to_cam Pose of the camera in lidar coordinates
     */
    bool Initialize(const std::shared_ptr<CameraPinhole>& camera);

    bool Initialize(const Eigen::Matrix3d &K);
    bool Initialize(const Eigen::Matrix3d &K, const Eigen::VectorXd& d);

    /*
     * Initializes the parameters from a config file. Must be called before class usuage.
     */
    bool InitConfig(const std::string& filePath, const bool printparams = true);

    /*
     * Initializes the parameters with default parameters. Must be called before class usuage.
     */
    bool InitConfig(const bool printparams = false);

    /*
     * Sets the lidar pointcloud and transforms the included points into camera coordinates
     * @param pointCloud pointcloud from velodyne in lidar cs
     */
    void setInputCloud(const Cloud::ConstPtr& pointCloud, const Eigen::Vector4d& qCL_WXYZ, const Eigen::Vector3d& rCL);

    /*
     * Sets the lidar pointcloud and transforms the included points into camera coordinates
     * @param pointCloud pointcloud from velodyne in lidar cs
     */
    void setInputCloud(const Cloud::ConstPtr& pointCloud);

    /*
     * Gets the parameter object
     */
    std::shared_ptr<DepthEstimatorParameters> getParameters() {
        return _parameters;
    }

    /*
     * Gets the camera object
     */
    std::shared_ptr<CameraPinhole> getCamera() {
        return _camera;
    }

    double getPointDepthCamVisible(int loopCountIndex) {
      return norm(_points._points_cs_camera[loopCountIndex]);
    }

    /*
     * Returns the original pointcloud in camera coordinates
     */
    void getCloudCameraCs(Cloud::Ptr& pointCloud_cam_cs);

    /*
     * Returns a point cloud containing all points which were found by the neirest neighbors search
     */
    void getCloudNeighbors(std::vector<cv::Point2f>& pts);

    /*
     * Gets the lidar point cloud points which are visible in the camera image frame in image cs.
     */
    void getPointsCloudImageCs(std::vector<cv::Point2f> &visiblePointsImageCs, std::vector<double> &depths);

    /*
     * gets the actual statistics of the last calculated frame
     */
    const DepthCalculationStatistics& getDepthCalcStats();

  /*
  * Calculates the depth of a set of given points
  * The pointcloud and the image must be synchronized in time
  * @param pointCloud [in] 3D pointcloud. Only points inside the camera view cone will be considered
  * @param points_image_cs [in] Points in image coordinates for which the depth will be calculated. Stored in row
  * order
  * @param points_depths [out] Depths of the given points in meters
  */
  void CalculateDepth(const Cloud::ConstPtr& pointCloud,
                      const Eigen::Matrix2Xd& points_image_cs,
                      Eigen::VectorXd& points_depths);

  void CalculateDepth(const Cloud::ConstPtr& pointCloud,
                      const Eigen::Matrix2Xd& points_image_cs,
                      Eigen::VectorXd& points_depths,
                      Eigen::VectorXi& resultType);

  std::pair<DepthResultType, double> CalculateDepth(
      const Eigen::Vector2d& featurePoint_image_cs,
      std::shared_ptr<DepthCalcStatsSinglePoint> calcStats);
  /*
   * Calculates the depth of a set of given points
   * @param points_image_cs [in] Points in image coordinates for which the depth will be calculated. Stored in row
   * order
   * @param points_depths [out] Depths of the given points in meters
   */
    void CalculateDepth(const Eigen::Matrix2Xd& points_image_cs,
                      Eigen::VectorXd& points_depths);

    void CalculateDepth(const Eigen::Matrix2Xd& points_image_cs,
                      Eigen::VectorXd& points_depths, Eigen::VectorXi& resultType);


    void LogDepthCalcStats(const DepthResultType depthResult);
private:
    bool InitializeParameters();

    /*
     * Creates a pointcloud with a list of 3D points
     *
     * @param content List of 3d points
     * @param cloud Created cloud
     */
    void FillCloud(const std::vector<Eigen::Vector3d> content, const Cloud::Ptr& cloud);

    /*
     * Gets the neighbors inside a rect in the image plane
     *
     * @param featurePoint_image_cs 2D feature point in the image plane
     * @param neighborIndices Found indices of the 2D-Neighbors. The indices are based on the visible points in the
     * image plane.
     * @param neighbors 3D-Lidar Positions of the Neighbor points
     * @param Object for statistics
     */
    bool CalculateNeighbors(const Eigen::Vector2d& featurePoint_image_cs,
                            std::vector<uint16_t>& neighborIndices,
                            std::vector<Eigen::Vector3d>& neighbors,
                            std::shared_ptr<DepthCalcStatsSinglePoint> calcStats,
                            const float scaleX = 1.0,
                            const float scaleY = 1.0);

    bool CalculateDepthSegmentation(const std::vector<Eigen::Vector3d>& neighbors,
                                    const std::vector<uint16_t>& neighborsIndex,
                                    std::vector<Eigen::Vector3d>& pointsSegmented,
                                    std::vector<int>& pointsSegmentedIndex,
                                    std::shared_ptr<DepthCalcStatsSinglePoint> calcStats);


    std::pair<DepthResultType, double> CalculateDepthSegmented(const Eigen::Vector2d& point_image_cs,
                                                               const std::vector<Eigen::Vector3d>& neighborsSegmented,
                                                               std::shared_ptr<DepthCalcStatsSinglePoint> calcStats,
                                                               const bool checkPlanarTriangle = false);

    /**
     * Transforms the pointcloud (lidar cs) into the camera cs and projects it's points into the image frame
     * @param lidar_to_cam Affine transformation from lidar to camera cooridnates#include <opencv2/core.hpp>
     */
    void Transform_Cloud_LidarToCamera(const Cloud::ConstPtr &cloud_lidar_cs,
        const cv::Mat_<float>& R_CL  = cv::Mat_<float>::eye(3,3),
        const cv::Mat_<float>& r_CL = cv::Mat_<float>::zeros(3,1));

    // Initialization flags
    /*
     * Flag which determines if the Initialize method has been called
     */
    bool _isInitialized = false;
    bool _isInitializedConfig = false;
    bool _isInitializedPointCloud = false;

    // pointcloud variables
    PointcloudData _points; // stored all points of the pointcloud in

    std::vector<cv::Point2f> _points_neighbors;   // list of points of all found neighbor points

    // Following objects are the modules used for depth estimation
    std::shared_ptr<NeighborFinderBase> _neighborFinder;

    std::shared_ptr<TresholdDepthGlobal> _tresholdDepthGlobal;
    std::shared_ptr<LinePlaneIntersectionBase> _linePlaneIntersection;
    std::shared_ptr<PlaneEstimationCheckPlanar> _checkPlanarTriangle;
    std::shared_ptr<PlaneEstimationCalcMaxSpanningTriangle> _planeCalcMaxSpanning;

    // Misc
    std::shared_ptr<DepthEstimatorParameters> _parameters;
    std::shared_ptr<CameraPinhole> _camera;

    DepthCalculationStatistics _depthCalcStats;
    const Eigen::Vector3d viewingRaySupportPoint = Eigen::Vector3d::Zero();

    // image variabes (input image)
    int _imgWitdh = 720;
    int _imgHeight = 480;
};


} /* namespace lidorb_ros_tool */
