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

  inline void quaternionToRotationMatrix(const cv::Mat_<float>& q_XYZW, cv::Mat_<float>& R)
  {
    float x = q_XYZW.at<float>(0);
    float y = q_XYZW.at<float>(1);
    float z = q_XYZW.at<float>(2);
    float w = q_XYZW.at<float>(3);

    const float tx  = 2.0f*x;
    const float ty  = 2.0f*y;
    const float tz  = 2.0f*z;
    const float twx = tx*w;
    const float twy = ty*w;
    const float twz = tz*w;
    const float txx = tx*x;
    const float txy = ty*x;
    const float txz = tz*x;
    const float tyy = ty*y;
    const float tyz = tz*y;
    const float tzz = tz*z;

    R = (cv::Mat_<float>(3,3) << 1.0f-(tyy+tzz), txy-twz, txz+twy,
                                 txy+twz, 1.0f-(txx+tzz), tyz-twx,
                                 txz-twy, tyz+twx, 1.0f-(txx+tyy));
  }

}
#endif //MONOLIDAR_FUSION_COMMON_H
