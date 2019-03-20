#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "monolidar_fusion/DepthEstimator.h"
#include <thread>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "monolidar_fusion/DepthEstimationWrapper.h"


std::mutex m_wrapper;
DepthEstimationWrapper wrapper;


void handleCloudMessage(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
  // fetch new input cloud
  pcl::PointCloud<pcl::PointXYZI> laser_cloud;
  pcl::fromROSMsg(*laserCloudMsg, laser_cloud);
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr lidar_cloud;
  lidar_cloud = pcl::PointCloud<pcl::PointXYZI>::ConstPtr(new pcl::PointCloud<pcl::PointXYZI>(laser_cloud));
  std::lock_guard<std::mutex> lock(m_wrapper);
  wrapper.lidar_time = laserCloudMsg->header.stamp.toSec();
  wrapper.handleDepthAssociation(lidar_cloud);
}

void handleFeatureCloudMessage(const sensor_msgs::PointCloud2ConstPtr &featureCloudMsg)
{
  std::lock_guard<std::mutex> lock(m_wrapper);
  // fetch new input cloud
  //pcl::PointCloud<pcl::PointXYZI> feature_cloud;
  pcl::fromROSMsg(*featureCloudMsg, *(wrapper.feature_point_cloud));
  //wrapper.feature_point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(new pcl::PointCloud<pcl::PointXYZI>(feature_cloud));
  wrapper.feature_time = featureCloudMsg->header.stamp.toSec();
  wrapper.assignValuesFromFeatureCloud();
}

/*void imgCallback(const sensor_msgs::ImageConstPtr & img){
  // Get image from msg
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_8UC1);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  std::lock_guard<std::mutex> lock(m_wrapper);
  cv_ptr->image.copyTo(wrapper.image);
  wrapper.image_time = img->header.stamp.toSec();
}*/

/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "fusion_node");
  ros::NodeHandle node;
  ros::NodeHandle nh_private("~");
  /*ros::Subscriber pcl_sub = node.subscribe("/lidar", 10, handleCloudMessage);
  ros::Subscriber features_sub = node.subscribe("/features", 10, handleFeatureCloudMessage);
  ros::Subscriber image_sub = node.subscribe("/image", 10, imgCallback);


  while (ros::ok()) {
    ros::spinOnce();
  }*/

  rosbag::Bag bagIn;
  std::string rosbag_filename = "dataset.bag";
  nh_private.param("rosbag_filename", rosbag_filename, rosbag_filename);

  bagIn.open(rosbag_filename, rosbag::bagmode::Read);

  std::string lidar_topic_name="/jay/os1_points_filtered";
  std::string feature_topic_name="/jay/rovio/pcl";
  std::string image_topic_name="/jay/cam0/image_raw";
  std::vector<std::string> topics;
  topics.push_back(feature_topic_name);
  topics.push_back(image_topic_name);
  topics.push_back(lidar_topic_name);
  rosbag::View view(bagIn, rosbag::TopicQuery(topics));

  for(rosbag::View::iterator it = view.begin();it != view.end() && ros::ok();it++){
    if(it->getTopic() == feature_topic_name){
      sensor_msgs::PointCloud2::ConstPtr feature_msg = it->instantiate<sensor_msgs::PointCloud2>();
      if (feature_msg != NULL) handleFeatureCloudMessage(feature_msg);
    }
    /*else if(it->getTopic() == image_topic_name){
      sensor_msgs::ImageConstPtr imgMsg = it->instantiate<sensor_msgs::Image>();
      if (imgMsg != NULL) imgCallback(imgMsg);
    }*/
    else if(it->getTopic() == lidar_topic_name){
      sensor_msgs::PointCloud2::ConstPtr lidar_msg = it->instantiate<sensor_msgs::PointCloud2>();
      if (lidar_msg != NULL) handleCloudMessage(lidar_msg);
    }
    ros::spinOnce();
  }

  bagIn.close();

  return 0;
}
