// Copyright 2018. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

/**
* @class CameraPinhole
* @par
*
* interface for a pinhole camera
*/
class CameraPinhole {
 public:  // Public methods.
  // Default constructor.
  explicit CameraPinhole(int width, int height, float focal_length_x,
                         float focal_length_y, float principal_point_x,
                         float principal_point_y,
                         const cv::Mat_<float>& distortion_coefficients =
                             cv::Mat_<float>(4, 1, 0.0f))
      : width_(width),
        height_(height),
        fx(focal_length_x),
        fy(focal_length_y),
        cx(principal_point_x),
        cy(principal_point_y) {
    intrinsics = cv::Mat_<float>(3, 3);
    intrinsics << focal_length_x, 0, principal_point_x, 0, focal_length_y,
        principal_point_y, 0, 0, 1;
    std::cout << std::endl
              << "Setting LIMO camera matrix to:" << std::endl
              << intrinsics << std::endl;
    distortion_coeffs = distortion_coefficients;
    std::cout << std::endl
              << "Setting LIMO distortion coeffs to:" << std::endl
              << distortion_coeffs << std::endl;
  }

  explicit CameraPinhole(int width, int height,
                         const cv::Mat_<float>& camera_matrix,
                         const cv::Mat_<float>& distortion_coefficients =
                             cv::Mat_<float>(4, 1, 0.0f))
      : width_(width), height_(height) {
    intrinsics = camera_matrix;
    fx = intrinsics.at<float>(0, 0);
    fy = intrinsics.at<float>(1, 1);
    cx = intrinsics.at<float>(0, 2);
    cy = intrinsics.at<float>(1, 2);
    distortion_coeffs = distortion_coefficients;
    std::cout << std::endl
              << "Setting LIMO camera matrix to:" << std::endl
              << intrinsics << std::endl;
    std::cout << std::endl
              << "Setting LIMO distortion coeffs to:" << std::endl
              << distortion_coeffs << std::endl;
  }

  // Default destructor.
  ~CameraPinhole() = default;

  // Default move.
  CameraPinhole(CameraPinhole&& other) = default;
  CameraPinhole& operator=(CameraPinhole&& other) = default;

  // Default copy.
  CameraPinhole(const CameraPinhole& other) = default;
  CameraPinhole& operator=(const CameraPinhole& other) = default;

  void getImageSize(int& width, int& height) const {
    width = width_;
    height = height_;
  }

  void getViewingRays(const Eigen::Vector2d& image_point,
                      Eigen::Vector3d& direction) const {
    std::vector<cv::Point2f> pts, undistorted;
    pts.push_back(cv::Point2f(image_point[0], image_point[1]));
    cv::undistortPoints(pts, undistorted, intrinsics, distortion_coeffs,
                        cv::noArray(), intrinsics);
    const cv::Point2f& undistorted_pt = undistorted[0];
    direction[0] = (undistorted_pt.x - cx) / fx;
    direction[1] = (undistorted_pt.y - cy) / fy;
    direction[2] = 1.0;
  }

  void getImagePoints(
      const std::vector<cv::Point3f>& lidar_points,
      std::vector<cv::Point2f>& image_points,
      const cv::Mat_<float>& R_CL = cv::Mat_<float>::eye(3, 3),
      const cv::Mat_<float>& r_CL = cv::Mat_<float>::zeros(3, 1)) const {
    cv::Mat_<float> rotVec_CL(3, 1);
    cv::Rodrigues(R_CL, rotVec_CL);
    cv::projectPoints(lidar_points, rotVec_CL, r_CL, intrinsics,
                      distortion_coeffs, image_points);
  }

 private:
  int width_;
  int height_;
  float fx;
  float fy;
  float cx;
  float cy;
  cv::Mat_<float> intrinsics;
  cv::Mat_<float> distortion_coeffs;
};
