//
// Created by nico on 20.03.19.
//

#ifndef MONOLIDAR_FUSION_COMMON_H
#define MONOLIDAR_FUSION_COMMON_H

#include <opencv2/core.hpp>

namespace Mono_Lidar {
  inline double norm(const cv::Point3f& pt)
  {
    return sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
  }

}
#endif //MONOLIDAR_FUSION_COMMON_H
