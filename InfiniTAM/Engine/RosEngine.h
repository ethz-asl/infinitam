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

#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "../ITMLib/Utils/ITMLibDefines.h"

// #endif

namespace InfiniTAM {
namespace Engine {
class RosEngine : public ImageSourceEngine {
 private:
  cv_bridge::CvImagePtr cv_rgb_image_;
  cv_bridge::CvImagePtr cv_depth_image_;
  //! True if the RGB data is available.
  bool rgb_ready_;
  //! True if the Depth data is available.
  bool depth_ready_;
  //! True if a tf message was received.
  bool first_time_tf_available_;
  //! True if one wants to broadcast the transforms calculated here.
  bool broadcast_transformations;
  //! True if the Depth info is available.
  bool rgb_info_ready_;
  //! True if the Depth info is available.
  bool depth_info_ready_;

  bool data_available_;
  //! ROS topic name for the incoming rgb messages.
  std::string rgb_camera_info_topic_;
  //! ROS Topic name for the incoming depth messages.
  std::string depth_camera_info_topic_;
  //! Name for the depth camera frame id in TF.
  std::string camera_frame_id_;
  //! Name for the fixed frame in TF.
  std::string world_frame_id_;
  //! ROS topic name where the generated complete cloud is published.
  std::string complete_cloud_topic_;
  std::mutex rgb_mutex_;
  std::mutex depth_mutex_;
  Vector2i image_size_rgb_, image_size_depth_;
  sensor_msgs::CameraInfo rgb_info_;
  sensor_msgs::CameraInfo depth_info_;
  //! ROS service name, when called the current mesh is transformed into a point cloud and published.
  ros::ServiceServer publish_scene_service_;
  //! ROS publisher to send out the complete cloud.
  ros::Publisher complete_point_cloud_pub_;

  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;

  // Infinitam Vector, represents translation from the infititam origin to the camera pose.
  Vector3f infinitam_translation_vector_;
  // Infinitam Matrix, represents rotation from the infititam origin to the camera pose.
  Matrix3f infinitam_rotation_matrix_;

  tf::StampedTransform tf_world_to_camera_transform_current_;
  tf::StampedTransform tf_world_to_camera_transform_at_start_;
  tf::StampedTransform tf_infinitam_origin_to_camera_transform_relative_;
  tf::Matrix3x3 tf_infinitam_origin_to_camera_current_rotation;
  tf::Vector3 tf_infinitam_origin_to_camera_current_translation_in_world_frame,
      tf_infinitam_origin_to_camera_current_translation_in_infinitam_frame,
      tf_infinitam_origin_to_camera_current_translation_in_infinitam_frame_rotated;
 public:
  RosEngine(ros::NodeHandle& nh, const char*& calibration_filename);

  ~RosEngine();
  void rgbCallback(const sensor_msgs::Image::ConstPtr& msg);
  void rgbCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void depthCallback(const sensor_msgs::Image::ConstPtr& msg);
  void depthCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void TFCallback(const tf::tfMessage &tf_msg);

  // ImageSourceEngine
  bool hasMoreImages(void);
  void getImages(ITMUChar4Image*rgb, ITMShortImage* raw_depth);
  Vector2i getDepthImageSize(void);
  Vector2i getRGBImageSize(void);

  //! Get mesh from Main Engine and return pcl pointer.
  void extractMeshToPcl(pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud);

  //! ROS Service Callback method.
  bool publishMap(std_srvs::Empty::Request& request,
      std_srvs::Empty::Response& response);
    };
  }
  // namespace Engine
  }// namespace InfiniTAM
