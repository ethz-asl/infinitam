// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ImageSourceEngine.h"
#include "PoseSourceEngine.h"
#include <glog/logging.h>

#include <mutex>
#include <string>
#if (!defined USING_CMAKE) && (defined _MSC_VER)
#ifdef _DEBUG
#pragma comment(lib, "libpxcmd_d")
#else
#pragma comment(lib, "libpxcmd")
#endif
#endif
// #ifdef COMPILE_WITH_Ros
#include <cv_bridge/cv_bridge.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>  // transformPointCloud
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include "../ITMLib/Utils/ITMLibDefines.h"

// #endif

namespace InfiniTAM {
namespace Engine {
class RosEngine : public ImageSourceEngine, public PoseSourceEngine {
 private:
  cv_bridge::CvImagePtr cv_rgb_image_;
  cv_bridge::CvImagePtr cv_depth_image_;
  bool rgb_ready_;
  bool depth_ready_;
  bool tf_ready_;
  bool rgb_info_ready_;
  bool depth_info_ready_;
  bool data_available_, tf_available_;
  bool debug_mode_;
  std::string rgb_camera_info_topic_;
  std::string depth_camera_info_topic_;
  std::string camera_frame_id_;
  std::string world_frame_id_;
  std::string complete_cloud_topic_;
  std::mutex rgb_mutex_;
  std::mutex depth_mutex_;
  std::mutex tf_mutex_;
  Vector2i image_size_rgb_, image_size_depth_;
  sensor_msgs::CameraInfo rgb_info_;
  sensor_msgs::CameraInfo depth_info_;
  ITMPose* camera_pose_;
  ros::Publisher marker_pub_;

  // create a ROS transformation listener
  tf::TransformListener listener;
  ros::Publisher complete_point_cloud_pub_;
  ros::ServiceServer publish_scene_service_;
  tf::StampedTransform camera_base_transform_;

 public:
  RosEngine(ros::NodeHandle& nh, const char*& calibration_filename);

  ~RosEngine();
  void rgbCallback(const sensor_msgs::Image::ConstPtr& msg);
  void rgbCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void depthCallback(const sensor_msgs::Image::ConstPtr& msg);
  void depthCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void TFCallback(const tf::tfMessage &tf_msg);

  // PoseSourceEngine
  void getMeasurement(ITMPose* pose);
  bool hasMoreMeasurements(void);

  // ImageSourceEngine
  bool hasMoreImages(void);
  void getImages(ITMUChar4Image* rgb, ITMShortImage* raw_depth);
  Vector2i getDepthImageSize(void);
  Vector2i getRGBImageSize(void);

  // get mesh from Main Engine and return ROS PointCloud2
  void extractMeshToPcl(pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud);
  // ROS Service Callback method which published the mesh as PointCloud
  bool publishMap(std_srvs::Empty::Request& request,
                  std_srvs::Empty::Response& response);
};
}  // namespace Engine
}  // namespace InfiniTAM
