/*
 * NeighborFinderPixel.h
 *
 *  Created on: Feb 6, 2017
 *      Author: wilczynski
 */

#pragma once

#include <Eigen/Eigen>
#include <opencv2/core/types.hpp>
#include <opencv2/core.hpp>
#include "NeighborFinderBase.h"

namespace Mono_Lidar {
class NeighborFinderPixel : public NeighborFinderBase {
public:
    NeighborFinderPixel(const int imgWitdh,
                        const int imgHeight,
                        const int pixelSearchWidth,
                        const int pixelSearchHeight);

    void Initialize(std::shared_ptr<DepthEstimatorParameters>& parameters) override;

    void InitializeLidarProjection(const std::vector<cv::Point2f> &lidarPoints_image_cs);

    // gets the neighbors of a given feature point including the point itself
    void getNeighbors(const Eigen::Vector2d &featurePoint_image_cs,
                      const std::vector<cv::Point3f> &points_cs_camera,
                      std::vector<uint16_t> &pcIndicesCut,
                      const std::shared_ptr<DepthCalcStatsSinglePoint> &calcStats = NULL,
                      const float scaleWidth = 1,
                      const float scaleHeight = 1) override;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    cv::Mat _img_points_lidar; // Matrix with dim (imgWitdh*imgHeight). Each pixels stores a index of a
                                       // projected (visible) lidar point or -1 if no point aviable at this pixel

    int _imgWitdth;
    int _imgHeight;

    int _pixelSearchWidth;
    int _pixelSearchHeight;
};
}
