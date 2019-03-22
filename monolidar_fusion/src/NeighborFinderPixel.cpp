/*
 * NeighborFinderPixel.cpp
 *
 *  Created on: Feb 6, 2017
 *      Author: wilczynski
 */

#include "monolidar_fusion/NeighborFinderPixel.h"
#include "monolidar_fusion/Logger.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace Mono_Lidar {

NeighborFinderPixel::NeighborFinderPixel(const int imgWitdh,
                                         const int imgHeight,
                                         const int pixelSearchWidth,
                                         const int pixelSearchHeight)
        : _imgWitdth(imgWitdh), _imgHeight(imgHeight), _pixelSearchWidth(pixelSearchWidth),
          _pixelSearchHeight(pixelSearchHeight) {
    _img_points_lidar = cv::Mat::zeros(imgHeight, imgWitdh, CV_16U);
}

void NeighborFinderPixel::Initialize(std::shared_ptr<DepthEstimatorParameters>& parameters) {
    NeighborFinderBase::Initialize(parameters);

    _pixelSearchWidth = _parameters->pixelarea_search_witdh;
    _pixelSearchHeight = _parameters->pixelarea_search_height;
}

void NeighborFinderPixel::InitializeLidarProjection(const std::vector<cv::Point2f> &lidarPoints_image_cs) {
    Logger::Instance().Log(Logger::MethodStart, "DepthEstimator::TransformLidarPointsToImageFrame");

    // todo: nico this currently rejects the upper left corner pixel
    for (uint16_t i=0; i < lidarPoints_image_cs.size();i++)
    {
        const cv::Point2f pt = lidarPoints_image_cs[i];
        _img_points_lidar.at<uint16_t>(pt) = i;
        //coloredImage.at<cv::Vec3b>(pt) = cv::Vec3b(255,255,255);
    }
    Logger::Instance().Log(Logger::MethodEnd, "DepthEstimator::TransformLidarPointsToImageFrame");
}

void NeighborFinderPixel::getNeighbors(const Eigen::Vector2d &featurePoint_image_cs,
                                       const std::vector<cv::Point3f> &points_cs_camera,
                                       std::vector<uint16_t> &pcIndicesCut,
                                       const std::shared_ptr<DepthCalcStatsSinglePoint> &calcStats,
                                       const float scaleWidth,
                                       const float scaleHeight) {
    double halfSizeX = static_cast<double>(_pixelSearchWidth) * 0.5 * static_cast<double>(scaleWidth);
    double halfSizeY = static_cast<double>(_pixelSearchHeight) * 0.5 * static_cast<double>(scaleHeight);

    double leftEdgeX = std::max(featurePoint_image_cs[0] - halfSizeX, 0.);
    double rightEdgeX = std::min(featurePoint_image_cs[0] + halfSizeX, static_cast<double>(_imgWitdth - 1));
    double topEdgeY = std::max(featurePoint_image_cs[1] - halfSizeY, 0.);
    double bottomEdgeY = std::min(featurePoint_image_cs[1] + halfSizeY, static_cast<double>(_imgHeight - 1));

    for (int i = static_cast<int>(topEdgeY); i <= static_cast<int>(bottomEdgeY); i++) {
        for (int j = static_cast<int>(leftEdgeX); j <= static_cast<int>(rightEdgeX); j++) {
            uint16_t index = _img_points_lidar.at<uint16_t>(i, j);
            if (index > 0u) {
                pcIndicesCut.push_back(index);
            }
        }
    }
    /*std::vector<cv::Point> pts;
    cv::Rect roi(featurePoint_image_cs[0], featurePoint_image_cs[1], _pixelSearchWidth, _pixelSearchHeight);
    cv::findNonZero(_img_points_lidar(roi), pts);
    for (auto pt : pts)
        std::cout << "\n" << pt.x << ", " << pt.y << "\n";*/

    // debug log neighbors finder
    if (calcStats != NULL) {
        calcStats->_searchRectTopLeft = std::pair<int, int>((int)leftEdgeX, (int)topEdgeY);
        calcStats->_searchRectBottomRight = std::pair<int, int>((int)rightEdgeX, (int)bottomEdgeY);
    }
}
}
