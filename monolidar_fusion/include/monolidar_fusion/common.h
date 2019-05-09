//
// Created by nico on 20.03.19.
//

#ifndef MONOLIDAR_FUSION_COMMON_H
#define MONOLIDAR_FUSION_COMMON_H

#include <Eigen/Eigen>
#include <opencv2/core.hpp>

namespace Mono_Lidar {
  inline double norm(const cv::Point3f& pt)
  {
    return sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
  }

  inline double normSquared(const cv::Point3f& pt)
  {
    return pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
  }

  inline void quaternionToRotationMatrix(float x, float y, float z, float w, cv::Mat_<float> &R) {
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

    R = (cv::Mat_<float>(3, 3) << 1.0f-(tyy+tzz), txy-twz, txz+twy,
        txy+twz, 1.0f-(txx+tzz), tyz-twx,
        txz-twy, tyz+twx, 1.0f-(txx+tyy));
  }

  inline void quaternionToRotationMatrix(const Eigen::Vector4d& v_WXYZ, cv::Mat_<float>& R)
  {
    float x = static_cast<float>(v_WXYZ[1]);
    float y = static_cast<float>(v_WXYZ[2]);
    float z = static_cast<float>(v_WXYZ[3]);
    float w = static_cast<float>(v_WXYZ[0]);

    quaternionToRotationMatrix(x, y, z, w, R);

  }

  inline void quaternionToRotationMatrix(const cv::Mat_<float>& q_XYZW, cv::Mat_<float>& R)
  {
    float x = q_XYZW.at<float>(0);
    float y = q_XYZW.at<float>(1);
    float z = q_XYZW.at<float>(2);
    float w = q_XYZW.at<float>(3);

    quaternionToRotationMatrix(x, y, z, w, R);
  }

}
#endif //MONOLIDAR_FUSION_COMMON_H
