/*
 * NeighborFinder.h
 *
 *  Created on: Feb 6, 2017
 *      Author: wilczynski
 */

#pragma once

#include <memory>
#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#include "DepthCalcStatsSinglePoint.h"
#include "DepthEstimatorParameters.h"
#include <Eigen/StdVector>

namespace Mono_Lidar {
/*
 * Base class used for selecting lidar point neighbors around a feature point in the image plane.
 */
class NeighborFinderBase {
public:
    // pcIndicesRaw: Found indizes of the points which are stored in the full/not resized/original cloud matrix
    virtual void Initialize(std::shared_ptr<DepthEstimatorParameters>& parameters);


    /*
     * Get the neighbor indices of the pixels around a given target pixel
     * @featurePoint_image_cs [in] The 2d coordinates of the target pixel
     * @points_cs_camera [in] Complete pointcloud in camera coordinates
     * @pointIndex [in] Map which transforms the index of the points (seen by the camera) into the index of he complete
     * point set
     * @pcIndicesRaw [out] The indices of the neighbors based on cam pointcloud
     */
    virtual void getNeighbors(const Eigen::Vector2d &featurePoint_image_cs,
                              const std::vector<cv::Point3f> &points_cs_camera,
                              std::vector<uint16_t> &pcIndicesRaw,
                              const std::shared_ptr<DepthCalcStatsSinglePoint> &calcStats = NULL,
                              const float scaleWidth = 1,
                              const float scaleHeight = 1) = 0;

    /*
     * Get the 3d Points from a cloud using it's indices
     * @points_cs_camera [in] Complete pointcloud in camera coordinates
     * @param imgIndicesCut [in] Indices of the neighbors based on the cut image indices
     * @param neighbors [out] List of the 3d neighbors (lidar points)
     */
    void getNeighbors(const std::vector<cv::Point3f> &points_cs_camera,
                      const std::vector<uint16_t> &imgIndicesCut,
                      std::vector<Eigen::Vector3d> &neighbors);

    /*
     * Get the 3d Points from a cloud using it's indices
     * @points_cs_camera [in] Complete pointcloud in camera coordinates
     * @param pcIndices [in] Indices of the neighbors based on the full pointcloud
     */
    void getNeighbors(const Eigen::Matrix3Xd& points_cs_camera,
                      const std::vector<int>& pcIndices,
                      std::vector<Eigen::Vector3d>& neighbors);

    virtual ~NeighborFinderBase() = default;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
    std::shared_ptr<DepthEstimatorParameters> _parameters;
};
}
