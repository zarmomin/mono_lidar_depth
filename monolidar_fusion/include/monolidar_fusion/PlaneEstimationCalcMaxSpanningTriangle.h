/*
 * PlaneEstimationCalcMaxSpanningTriangle.h
 *
 *  Created on: Mar 15, 2017
 *      Author: wilczynski
 */

#pragma once

#include <vector>
#include <Eigen/Eigen>
#include <Eigen/StdVector>

namespace Mono_Lidar {
/*
 * Gets of a list of 3D points the three points which span a triangle with a maximum size
 */
class PlaneEstimationCalcMaxSpanningTriangle {
public:
    PlaneEstimationCalcMaxSpanningTriangle(const bool publishPoints = false);

    /*
     * @param distTreshold The minimum distance (in meters) which 2 mutual points must have
     */
    PlaneEstimationCalcMaxSpanningTriangle(const double distTreshold, const bool publishPoints = false);

    /**
     * Selects three corner points from a point-list considering the edge condition to maximize the area of the spanning
     * triangle
     * @param points [in] list of corner point cantidates
     * @param corner1 [out] corner 1 of the biggest spanning triangle
     * @param corner2 [out] corner 2 of the biggest spanning triangle
     * @param corner3 [out] corner 3 of the biggest spanning triangle
     */
    bool CalculatePlaneCorners(const std::vector<Eigen::Vector3d>& points,
                               int& corner1,
                               int& corner2,
                               int& corner3);

    double area(int a, int b, int c, const std::vector<Eigen::Vector3d>& list);

    double area(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c);

private:
    double _distTreshold;
    bool _publishPoints;
};
}
