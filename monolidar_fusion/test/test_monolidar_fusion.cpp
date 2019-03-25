// google test docs
// wiki page: https://code.google.com/p/googletest/w/list
// primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
// FAQ: https://code.google.com/p/googletest/wiki/FAQ
// advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
// samples: https://code.google.com/p/googletest/wiki/V1_7_Samples
//
// List of some basic tests fuctions:
// Fatal assertion                      Nonfatal assertion                   Verifies / Description
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// ASSERT_EQ(expected, actual);         EXPECT_EQ(expected, actual);         expected == actual
// ASSERT_NE(val1, val2);               EXPECT_NE(val1, val2);               val1 != val2
// ASSERT_LT(val1, val2);               EXPECT_LT(val1, val2);               val1 < val2
// ASSERT_LE(val1, val2);               EXPECT_LE(val1, val2);               val1 <= val2
// ASSERT_GT(val1, val2);               EXPECT_GT(val1, val2);               val1 > val2
// ASSERT_GE(val1, val2);               EXPECT_GE(val1, val2);               val1 >= val2
//
// ASSERT_FLOAT_EQ(expected, actual);   EXPECT_FLOAT_EQ(expected, actual);   the two float values are almost equal (4
// ULPs)
// ASSERT_DOUBLE_EQ(expected, actual);  EXPECT_DOUBLE_EQ(expected, actual);  the two double values are almost equal (4
// ULPs)
// ASSERT_NEAR(val1, val2, abs_error);  EXPECT_NEAR(val1, val2, abs_error);  the difference between val1 and val2
// doesn't exceed the given absolute error
//
// Note: more information about ULPs can be found here:
// http://www.cygnus-software.com/papers/comparingfloats/comparingfloats.htm
//
// Example of two unit test:
// TEST(Math, Add) {
//    ASSERT_EQ(10, 5+ 5);
//}
//
// TEST(Math, Float) {
//	  ASSERT_FLOAT_EQ((10.0f + 2.0f) * 3.0f, 10.0f * 3.0f + 2.0f * 3.0f)
//}
//=======================================================================================================================================================
#include "gtest/gtest.h"

#include <Eigen/Eigen>

#include <memory>
#include <vector>

// test classes
#include "monolidar_fusion/HelperLidarRowSegmentation.h"
#include "monolidar_fusion/HistogramPointDepth.h"
#include "monolidar_fusion/NeighborFinderPixel.h"
#include "monolidar_fusion/PointcloudData.h"
#include "monolidar_fusion/DepthEstimator.h"
#include "monolidar_fusion/camera_pinhole.h"
#include "monolidar_fusion/DepthEstimatorParameters.h"
#include "monolidar_fusion/DepthEstimationWrapper.h"
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <regex>

// A google test function (uncomment the next function, add code and
// change the names TestGroupName and TestName)
// TEST(TestGroupName, TestName) {
// TODO: Add your test code here
//}

namespace {

Eigen::Vector2d project(Eigen::Vector3d p, Eigen::Matrix3d intrinsics) {
    Eigen::Vector3d proj = intrinsics * p;
    proj /= proj[2];

    return Eigen::Vector2d(proj[0], proj[1]);
}

Eigen::Vector3d projectKeepDepth(Eigen::Vector3d p, Eigen::Matrix3d intrinsics) {
    Eigen::Vector3d proj = intrinsics * p;
    proj /= proj[2];

    return Eigen::Vector3d(proj[0], proj[1], p.z());
}
}

void readPointCloud(const std::string& filename, pcl::PointCloud<pcl::PointXYZI>& cloud)
{
  std::ifstream file (filename); //file just has some sentences
  if (!file) {
    std::cout << "unable to open file";
    return;
  }
  std::string sentence;
  std::regex r("(.*) (.*) (.*) (.*)");
  int i=0;
  while (std::getline(file, sentence))
  {
    if (i>10) {
      std::smatch sm;
      if (std::regex_search(sentence, sm, r)) {
        pcl::PointXYZI pt;
        pt.x = std::strtof(sm[1].str().c_str(), 0);
        pt.y = std::strtof(sm[2].str().c_str(), 0);
        pt.z = std::strtof(sm[3].str().c_str(), 0);
        pt.intensity = std::strtof(sm[4].str().c_str(), 0);
        cloud.push_back(pt);
      }
    }
    i++;
  }
}

TEST(Interface, real_data)
{
  DepthEstimationWrapper wrapper;
  wrapper.lidar_time = 1549469580.744550;
  //wrapper.image_time = wrapper.lidar_time;
  wrapper.feature_time = wrapper.lidar_time;
  //wrapper.image = cv::imread("/home/nico/datasets/eschlikon/raww/1549469580.744550_img.png", cv::IMREAD_GRAYSCALE);
  pcl::PointCloud<pcl::PointXYZI> feature_cloud, laser_cloud;
  readPointCloud("/home/nico/datasets/eschlikon/raww/1549469580.744550_features.pcd", feature_cloud);
  *wrapper.feature_point_cloud = feature_cloud;
  readPointCloud("/home/nico/datasets/eschlikon/raww/1549469580.744550_cloud.pcd", laser_cloud);
  wrapper.assignValuesFromFeatureCloud();
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr lidar_cloud;
  lidar_cloud = pcl::PointCloud<pcl::PointXYZI>::ConstPtr(new pcl::PointCloud<pcl::PointXYZI>(laser_cloud));
  wrapper.handleDepthAssociation(lidar_cloud);
}

TEST(Interface, complete_run)
{
  const int img_width = 720;
  const int img_height = 480;
    const float cam_p_u = 375;
    const float cam_p_v = 239;
    const float cam_f = 395;
    std::srand(42);
    std::shared_ptr<CameraPinhole> cam;
    cam = std::make_shared<CameraPinhole>(img_width, img_height, cam_f, cam_f, cam_p_u, cam_p_v);
    Mono_Lidar::DepthEstimator depthEstimator;
    depthEstimator.InitConfig("/home/nico/catkin_ws/src/mono_lidar_depth/monolidar_fusion/parameters.yaml", false);
    depthEstimator.Initialize(cam);

    // generate camera points
    const int point_count = 50;

    Eigen::Matrix2Xd points_2d_orig;
    points_2d_orig.resize(2, point_count);

    for (int i = 0; i < point_count; i++) {
        const int u = ( std::rand() % ( img_width + 1 ) );
        const int v = ( std::rand() % ( img_height + 1 ) );
        points_2d_orig(0, i) = u;
        points_2d_orig(1, i) = v;
    }

    // generate lidar points
    double points_min_x = 4.99;
    double points_max_x = 5;
    double points_delta_x = 1;
    double points_min_y = -5;
    double points_max_y = 5;
    double points_delta_y = 0.04;
    double points_min_z = -3;
    double points_max_z = 3;
    double points_delta_z = 0.05;
    double intensity = 0;

    Eigen::VectorXd point_depths(point_count);

    pcl::PointCloud<pcl::PointXYZI> pointcloud;
    for (double y = points_min_y; y < points_max_y; y += points_delta_y) {
        for (double x = points_max_x; x > points_min_x; x -= points_delta_x) {
            for (double z = points_min_z; z < points_max_z; z+=points_delta_z) {
                pcl::PointXYZI pt(intensity);
                pt.x = -y;
                pt.y = -z;
                pt.z = x;
                pointcloud.push_back(pt);
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZI>::ConstPtr pointcloudpointer(new pcl::PointCloud<pcl::PointXYZI>(pointcloud));
    Mono_Lidar::GroundPlane::Ptr groundplane = nullptr;
    depthEstimator.CalculateDepth(pointcloudpointer, points_2d_orig, point_depths, groundplane);
    std::cout << "\n" << (point_depths.array() < 0).count() << "\n";
    // all points were successfully given a depth estimate
    ASSERT_TRUE(point_depths.minCoeff() >= points_min_x);
    ASSERT_TRUE(point_depths.maxCoeff() >= points_max_x);
}

TEST(Interface, complete_run2)
{
  const int img_width = 720;
  const int img_height = 480;
    const double cam_p_u = 375;
    const double cam_p_v = 239;
    const double cam_f = 395;
    std::srand(42);
    Eigen::Matrix3d intrinsics = Eigen::Matrix3d::Identity();
    intrinsics(0, 0) = intrinsics(1, 1) = cam_f;
    intrinsics(0, 2) = cam_p_u;
    intrinsics(1, 2) = cam_p_v;
    std::shared_ptr<Mono_Lidar::DepthEstimator> depthEstimator = std::make_shared<Mono_Lidar::DepthEstimator>();
    depthEstimator->InitConfig("/home/nico/catkin_ws/src/mono_lidar_depth/monolidar_fusion/parameters.yaml", false);
    depthEstimator->Initialize(intrinsics);

    // generate camera points
    const int point_count = 50;

    Eigen::Matrix2Xd points_2d_orig;
    points_2d_orig.resize(2, point_count);

    for (int i = 0; i < point_count; i++) {
        const int u = ( std::rand() % ( img_width + 1 ) );
        const int v = ( std::rand() % ( img_height + 1 ) );
        points_2d_orig(0, i) = u;
        points_2d_orig(1, i) = v;
    }

    // generate lidar points
    double points_min_x = 4.99;
    double points_max_x = 5;
    double points_delta_x = 1;
    double points_min_y = -5;
    double points_max_y = 5;
    double points_delta_y = 0.04;
    double points_min_z = -3;
    double points_max_z = 3;
    double points_delta_z = 0.05;
    double intensity = 0;

    Eigen::VectorXd point_depths(point_count);

    pcl::PointCloud<pcl::PointXYZI> pointcloud;
    for (double y = points_min_y; y < points_max_y; y += points_delta_y) {
        for (double x = points_max_x; x > points_min_x; x -= points_delta_x) {
            for (double z = points_min_z; z < points_max_z; z+=points_delta_z) {
                pcl::PointXYZI pt(intensity);
                pt.x = -y;
                pt.y = -z;
                pt.z = x;
                pointcloud.push_back(pt);
            }
        }
    }
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr pointcloudpointer(new pcl::PointCloud<pcl::PointXYZI>(pointcloud));
    //Mono_Lidar::GroundPlane::Ptr groundplane = nullptr;
    depthEstimator->CalculateDepth(pointcloudpointer, points_2d_orig, point_depths);
}

TEST(Interface, visiblePointRetrieval)
{
  const int img_width = 720;
  const int img_height = 480;
  const double cam_p_u = 375;
  const double cam_p_v = 239;
  const double cam_f = 395;
  std::srand(42);
  Eigen::Matrix3d intrinsics = Eigen::Matrix3d::Identity();
  intrinsics(0, 0) = intrinsics(1, 1) = cam_f;
  intrinsics(0, 2) = cam_p_u;
  intrinsics(1, 2) = cam_p_v;
  std::shared_ptr<Mono_Lidar::DepthEstimator> depthEstimator = std::make_shared<Mono_Lidar::DepthEstimator>();
  depthEstimator->InitConfig("/home/nico/catkin_ws/src/mono_lidar_depth/monolidar_fusion/parameters.yaml", false);
  depthEstimator->Initialize(intrinsics);

  // generate camera points
  const int point_count = 50;

  Eigen::Matrix2Xd points_2d_orig;
  points_2d_orig.resize(2, point_count);

  for (int i = 0; i < point_count; i++) {
    const int u = ( std::rand() % ( img_width + 1 ) );
    const int v = ( std::rand() % ( img_height + 1 ) );
    points_2d_orig(0, i) = u;
    points_2d_orig(1, i) = v;
  }

  // generate lidar points
  double points_min_x = 4.99;
  double points_max_x = 5;
  double points_delta_x = 1;
  double points_min_y = -5;
  double points_max_y = 5;
  double points_delta_y = 0.04;
  double points_min_z = -3;
  double points_max_z = 3;
  double points_delta_z = 0.05;
  double intensity = 0;

  Eigen::VectorXd point_depths(point_count);

  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  for (double y = points_min_y; y < points_max_y; y += points_delta_y) {
    for (double x = points_max_x; x > points_min_x; x -= points_delta_x) {
      for (double z = points_min_z; z < points_max_z; z+=points_delta_z) {
        pcl::PointXYZI pt(intensity);
        pt.x = -y;
        pt.y = -z;
        pt.z = x;
        pointcloud.push_back(pt);
      }
    }
  }
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr pointcloudpointer(new pcl::PointCloud<pcl::PointXYZI>(pointcloud));
  //Mono_Lidar::GroundPlane::Ptr groundplane = nullptr;
  depthEstimator->CalculateDepth(pointcloudpointer, points_2d_orig, point_depths);
  std::vector<cv::Point2f> visiblePoints;
  std::vector<double> depths;
  depthEstimator->getPointsCloudImageCs(visiblePoints, depths);
  ASSERT_TRUE(depths.size() > 0);
  ASSERT_TRUE(depths.size() == visiblePoints.size());

  for (int i=0;i < depths.size();i++)
  {
    ASSERT_TRUE(depths[i] > points_min_x);
    ASSERT_TRUE(visiblePoints[i].x >= 0 && visiblePoints[i].x < img_width);
    ASSERT_TRUE(visiblePoints[i].y >= 0 && visiblePoints[i].y < img_height);
  }
}

TEST(NeigborFinder, findByPixel) {
    const int img_width = 100;
    const int img_height = 100;
    const double cam_p_u = 50;
    const double cam_p_v = 50;
    const double cam_f = 60;
    const int nf_search_width = 3;
    const int nf_search_height = 5;

    // init objects
    std::shared_ptr<CameraPinhole> cam;
    cam = std::make_shared<CameraPinhole>(img_width, img_height, cam_f, cam_f, cam_p_u, cam_p_v);
    Mono_Lidar::NeighborFinderPixel neighborFinder(img_width, img_height, nf_search_width, nf_search_height);

    // init points
    const int point_count = 50;

    std::vector<cv::Point2f> points_2d_orig;
    for (int i = 0; i < point_count; i++) {
        const int u = std::rand() % 10;
        const int v = std::rand() % 10;
        points_2d_orig.push_back(cv::Point2f(u, v));
    }

    std::vector<cv::Point3f> points_3d_cam;
    for (auto pt: points_2d_orig) {
      Eigen::Vector2d ptt(pt.x, pt.y);
      Eigen::Vector3d dir;
      cam->getViewingRays(ptt, dir);
      points_3d_cam.push_back(cv::Point3f(dir[0],dir[1],dir[2]));
    }

    /*std::vector<cv::Point3f> points_3d_cam;
    // init features depth
    for (int i = 0; i < int(point_count); ++i) {
        points_3d_cam.push_back(static_cast<float>((std::rand() % 10 + 1)) * directions[i]);
    }*/

    // project points on image plane again
    std::vector<cv::Point2f> points_2d_projected;
    cam->getImagePoints(points_3d_cam, points_2d_projected);

    neighborFinder.InitializeLidarProjection(points_2d_orig);

    // get neighbors from tested method
    for (int i = 0; i < point_count; i++) {
        Eigen::Vector2d feature;
        feature.x() = points_2d_orig[i].x;
        feature.y() = points_2d_orig[i].y;

        std::vector<uint16_t > index_out;
        neighborFinder.getNeighbors(feature, points_3d_cam, index_out);

        // test if neighbor pos is in tolerance distance from feature point
        for (const auto& index : index_out) {
            Eigen::Vector2d neighbor_projected;
            neighbor_projected.x() = points_2d_projected[index].x;
            neighbor_projected.y() = points_2d_projected[index].y;

            Eigen::Vector2d neighbor_calc;
            neighbor_calc.x() = points_2d_orig[index].x;
            neighbor_calc.y() = points_2d_orig[index].y;

            double points_dist = (neighbor_projected - neighbor_calc).norm();
            double dist_from_feature_u = fabs(neighbor_calc.x() - feature.x());
            double dist_from_feature_v = fabs(neighbor_calc.y() - feature.y());

            ASSERT_NEAR(points_dist, 0., 0.01);
            ASSERT_LE(dist_from_feature_u, std::ceil(static_cast<double>(nf_search_width) * 0.5) + 0.01);
            ASSERT_LE(dist_from_feature_v, std::ceil(static_cast<double>(nf_search_height) * 0.5) + 0.01);
        }
    }
}


TEST(Histogram, GetNearestPoint) {
    // init point-list
    const int points_count = 10;
    std::vector<Eigen::Vector3d> input_points;
    std::vector<uint16_t> points_index;
    Eigen::VectorXd input_depths;
    input_depths.resize(points_count);

    float depth = 5;
    for (int i = 0; i < points_count; i++) {
        input_points.push_back(Eigen::Vector3d(0, 0, depth));
        points_index.push_back(i);
        input_depths[i] = depth;
        depth += 0.5f;
    }

    // test method
    std::vector<Eigen::Vector3d> output;
    std::vector<int> output_index;
    Mono_Lidar::PointHistogram::GetNearestPoint(input_points, points_index, input_depths, output, output_index);

    // test results
    ASSERT_EQ(1, output.size());
    ASSERT_EQ(1, output_index.size());
    ASSERT_EQ(output.front(), input_points.front());
    ASSERT_EQ(output_index.front(), points_index.front());
}

// Test the method to find a local maximum using a depth segmenting histogram
TEST(Histogram, FilterPointsMinDistBlob) {
    std::vector<Eigen::Vector3d> input_points;

    // init points
    input_points.push_back(Eigen::Vector3d(0, 0, 2.2));
    input_points.push_back(Eigen::Vector3d(0, 0, 3.5));
    input_points.push_back(Eigen::Vector3d(0, 0, 4.2));
    input_points.push_back(Eigen::Vector3d(0, 0, 5.2));
    input_points.push_back(Eigen::Vector3d(0, 0, 5.2));
    input_points.push_back(Eigen::Vector3d(0, 0, 6.2));
    input_points.push_back(Eigen::Vector3d(0, 0, 7.2));
    input_points.push_back(Eigen::Vector3d(0, 0, 8.2));
    input_points.push_back(Eigen::Vector3d(0, 0, 8.3));
    input_points.push_back(Eigen::Vector3d(0, 0, 8.4));
    input_points.push_back(Eigen::Vector3d(0, 0, 9.2));
    input_points.push_back(Eigen::Vector3d(0, 0, 10.2));
    input_points.push_back(Eigen::Vector3d(0, 0, 10.5));
    int points_count = input_points.size();

    std::vector<uint16_t> points_index;
    Eigen::VectorXd input_depths;
    input_depths.resize(points_count);

    for (int i = 0; i < points_count; i++) {
        input_depths[i] = input_points.at(i).z();
        points_index.push_back(i);
    }

    // histogram values
    const double bin_width = 1;
    const int minimum_max_size = 3;
    std::vector<Eigen::Vector3d> output;
    std::vector<int> output_index;
    double higher_border;
    double lower_border;

    auto result = Mono_Lidar::PointHistogram::FilterPointsMinDistBlob(input_points,
                                                                      points_index,
                                                                      input_depths,
                                                                      bin_width,
                                                                      minimum_max_size,
                                                                      output,
                                                                      output_index,
                                                                      lower_border,
                                                                      higher_border);

    std::cout << "Lower border: " << lower_border << std::endl;
    std::cout << "Higher border: " << higher_border << std::endl;
    std::cout << "Output depths: ";

    for (const auto& p : output) {
        std::cout << p.z() << ", ";
    }

    std::cout << std::endl;

    ASSERT_EQ(result, true);

    // check asserts
    for (const auto& p : output) {
        ASSERT_GE(p.z(), lower_border);
        ASSERT_LE(p.z(), higher_border);
    }

    ASSERT_EQ(output.size(), 3);
    ASSERT_EQ(output.at(0).z(), 8.2);
    ASSERT_EQ(output.at(1).z(), 8.3);
    ASSERT_EQ(output.at(2).z(), 8.4);
}
