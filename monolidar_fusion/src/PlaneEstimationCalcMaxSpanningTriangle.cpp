/*
 * PlaneEstimationCalcMaxSpanningTriangle.cpp
 *
 *  Created on: Mar 15, 2017
 *      Author: wilczynski
 */

#include <cmath>
#include "monolidar_fusion/PlaneEstimationCalcMaxSpanningTriangle.h"
#include <iostream>


namespace Mono_Lidar {
PlaneEstimationCalcMaxSpanningTriangle::PlaneEstimationCalcMaxSpanningTriangle(const bool publishPoints)
        : _distTreshold(0), _publishPoints(publishPoints) {
}

PlaneEstimationCalcMaxSpanningTriangle::PlaneEstimationCalcMaxSpanningTriangle(const double distTreshold,
                                                                               const bool publishPoints)
        : _distTreshold(distTreshold), _publishPoints(publishPoints) {
}


bool PlaneEstimationCalcMaxSpanningTriangle::CalculatePlaneCorners(const std::vector<Eigen::Vector3d>& points,
                                                                   int& corner1,
                                                                   int& corner2,
                                                                   int& corner3) {
  int n = points.size();
  if (n < 3)
    return false;

  int A=0,B=1,C=2;
  corner1=0;
  corner2=1;
  corner3=2;

  while (true)
  {
    while (true)
    {
      while (area(A,B,C, points) < area(A,B,(C+1)%n, points))
      {
        C = (C+1)%n;
      }
      if (area(A,B,C, points) < area(A,(B+1)%n,C, points))
      {
        B = (B + 1) % n;
        continue;
      }
      else
        break;
    }
    if (area(A,B,C, points) > area(corner1,corner2,corner3, points))
    {
      corner1 = A;
      corner2 = B;
      corner3 = C;
    }
    A = (A+1)%n;
    if (A==B) B = (B+1)%n;
    if (B==C) C = (C+1)%n;
    if (A==0) break;
  }
  
  /*std::cout << "\ncorner1: " << corner1 << " " << vA.x << ", " << vA.y << ", " << vA.z;
  std::cout << "\ncorner2: " << corner2 << " " << vB.x << ", " << vB.y << ", " << vB.z;
  std::cout << "\ncorner2: " << corner3 << " " << vC.x << ", " << vC.y << ", " << vC.z << "\n";*/
  return true;
}



double PlaneEstimationCalcMaxSpanningTriangle::area(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c) {
  // using herons formula
  double xab = b[0] - a[0];
  double yab = b[1] - a[1];
  double zab = b[2] - a[2];
  double xac = c[0] - a[0];
  double yac = c[1] - a[1];
  double zac = c[2] - a[2];
  return 0.5 * sqrt(pow((yab*zac - zab*yac),2) + pow((zab*xac - xab*zac),2) + pow((xab*yac - yab*xac),2));

}
double PlaneEstimationCalcMaxSpanningTriangle::area(int a, int b, int c, const std::vector<Eigen::Vector3d>& list) {
  return area(list[a], list[b], list[c]);
}
}
