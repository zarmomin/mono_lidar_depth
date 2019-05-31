//
// Created by nico on 18.03.19.
//

#ifndef MONOLIDAR_FUSION_DEPTHESTIMATIONWRAPPER_H
#define MONOLIDAR_FUSION_DEPTHESTIMATIONWRAPPER_H

#include <ros/ros.h>
#include "monolidar_fusion/DepthEstimator.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/// > Exclusively used for testing and development
class DepthEstimationWrapper{
 public:
  cv::Mat image;
  Mono_Lidar::DepthEstimator depth_estimator;
  double image_time;
  double lidar_time;
  double feature_time;
  Eigen::Matrix2Xd feature_points;
  std::vector<double> rovio_depths;
  std::vector<int> rovio_indices;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> feature_point_cloud;

  DepthEstimationWrapper() {
    feature_point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    Eigen::Matrix3d intrinsics;
    intrinsics << 395.872620767, 0.0, 372.495185619,
        0.0, 395.786208753, 214.319312646,
        0.0, 0.0, 1.0;
    Eigen::VectorXd d_(4);
    d_<< -0.00453674705911, 0.00837778666974, 0.0272220038607, -0.0155084020782;
    depth_estimator.InitConfig("/home/nico/catkin_ws/src/mono_lidar_depth/monolidar_fusion/parameters.yaml", false);
    depth_estimator.Initialize(intrinsics, d_);
    image_time = -1;
    lidar_time = -2;
    feature_time = -3;
  };

  void assignValuesFromFeatureCloud()
  {
    feature_points.resize(2, feature_point_cloud->width);
    rovio_depths.clear();
    rovio_indices.clear();
    for(size_t i=0; i<feature_point_cloud->width;i++)
    {
      const pcl::PointXYZI& pt = feature_point_cloud->at(i);
      feature_points(0, i) = pt.x;
      feature_points(1, i) = pt.y;
      rovio_depths.push_back(pt.z);
      rovio_indices.push_back(static_cast<int>(pt.intensity));
    }
  }

  void handleDepthAssociation(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& lidar_cloud)
  {
    // if timing is right
    if (std::fabs(image_time - feature_time) < 0.01 && std::fabs(feature_time - lidar_time) < 0.05) {
      int n_features = feature_points.cols();
      Eigen::VectorXd depths(n_features);
      Eigen::VectorXi resultTypes(n_features);
      depth_estimator.CalculateDepth(lidar_cloud, feature_points, depths, resultTypes);

      /// evaluate
      cv::Mat coloredImage;
      cv::cvtColor(image, coloredImage, cv::COLOR_GRAY2BGR);

      // lidar points over image
      std::vector<cv::Point2f> visiblePoints;
      std::vector<double> lidar_depths;
      depth_estimator.getPointsCloudImageCs(visiblePoints, lidar_depths);
      for (int i=0;i < lidar_depths.size();i++)
      {
        int intensity = static_cast<int>(2.0*lidar_depths[i]);
        cv::ellipse(coloredImage, visiblePoints[i], cv::Size(2,2), 0, 0, 360, cv::Scalar(0,50,intensity), -1, 8, 0);
      }

      // draw the feature points in pink and their indices
      for (int i=0;i < n_features;i++)
      {
        if (depths[i] < 0) {
          cv::Scalar intensity = cv::Scalar(153, 50, 255);
          cv::Point2f pt;
          pt.x = static_cast<float>(feature_points(0, i));
          pt.y = static_cast<float>(feature_points(1, i));
          cv::ellipse(coloredImage, pt, cv::Size(3, 3), 0, 0, 360, intensity, -1, 8, 0);
          pt.x += 4;
          pt.y -= 4;
          cv::putText(coloredImage, std::to_string(rovio_indices[i]), pt, cv::FONT_HERSHEY_SIMPLEX, 0.6, intensity);
        }
        // print results for all of them
        std::cout << "\nidx: " << rovio_indices[i] << " result type: " << Mono_Lidar::getTextForDepthResultType(resultTypes[i]);
      }

      // feature neighbors in blue
      std::vector<cv::Point2f> neighbors;
      depth_estimator.getCloudNeighbors(neighbors);
      for (int i=0;i < neighbors.size();i++)
      {
        cv::Scalar intensity = cv::Scalar(204,50,0);
        cv::ellipse(coloredImage, neighbors[i], cv::Size(2,2), 0, 0, 360, intensity, -1, 8, 0);
      }

      // draw the points for which the depth was successfully calculated in green and their indices
      for (int i=0;i < n_features;i++)
      {
        if (depths[i] > 0) {
          cv::Scalar intensity = cv::Scalar(51, 255, 51);
          cv::Point2f pt;
          pt.x = static_cast<float>(feature_points(0, i));
          pt.y = static_cast<float>(feature_points(1, i));
          cv::ellipse(coloredImage, pt, cv::Size(3, 3), 0, 0, 360, intensity, -1, 8, 0);
          pt.x += 4;
          pt.y -= 4;
          cv::putText(coloredImage, std::to_string(rovio_indices[i]), pt, cv::FONT_HERSHEY_SIMPLEX, 0.6, intensity);
        }
      }

      cv::imshow("image", coloredImage);
      cv::waitKey(0);
    }
  }
};

#endif //MONOLIDAR_FUSION_DEPTHESTIMATIONWRAPPER_H
