/*
 * PointcloudData.h
 *
 *  Created on: Jul 18, 2017
 *      Author: wilczynski
 */

#pragma once

#include <math.h>
#include <Eigen/Eigen>
#include <opencv2/core/types.hpp>

namespace Mono_Lidar {
class PointcloudData {

private:
public:
    std::vector<cv::Point3f> _points_cs_camera; // lidar points in camera-coordinates
    std::vector<cv::Point2f> _points_cs_image; // lidar points in image coordinates

    PointcloudData() {
    }

    void Clear()
    {
        _points_cs_camera.clear();
        _points_cs_image.clear();
    }

    static double CalcDistanceSquared(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2) {
        return (pow(vec1.x() - vec2.x(), 2) + pow(vec1.y() - vec2.y(), 2) + pow(vec1.z() - vec2.z(), 2));
    }

    static double CalcDistanceSquared(const Eigen::Vector2d& vec1, const Eigen::Vector2d& vec2) {
        return (pow(vec1.x() - vec2.x(), 2) + pow(vec1.y() - vec2.y(), 2));
    }

    static double CalcDistance(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2) {
        return sqrt(pow(vec1.x() - vec2.x(), 2) + pow(vec1.y() - vec2.y(), 2) + pow(vec1.z() - vec2.z(), 2));
    }

    static double CalcDistance(const Eigen::Vector2d& vec1, const Eigen::Vector2d& vec2) {
        return sqrt(pow(vec1.x() - vec2.x(), 2) + pow(vec1.y() - vec2.y(), 2));
    }
};
}
