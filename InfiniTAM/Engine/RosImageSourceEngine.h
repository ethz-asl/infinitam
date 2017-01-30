// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ImageSourceEngine.h"

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

#include <tf/transform_broadcaster.h>
#include "../ITMLib/Utils/ITMLibDefines.h"

// #endif

namespace InfiniTAM {
namespace Engine {
class RosImageSourceEngine : public ImageSourceEngine {
 private:
  cv_bridge::CvImagePtr cv_rgb_image_;
  cv_bridge::CvImagePtr cv_depth_image_;
  //! True if the RGB data is available.
  bool rgb_ready_;
  //! True if the Depth data is available.
  bool depth_ready_;
  //! True if the Depth info is available.
  bool rgb_info_ready_;
  //! True if the Depth info is available.
  bool depth_info_ready_;

  bool data_available_;
  //! ROS topic name for the incoming rgb messages.
  std::string rgb_camera_info_topic_;
  //! ROS Topic name for the incoming depth messages.
  std::string depth_camera_info_topic_;
  std::mutex rgb_mutex_;
  std::mutex depth_mutex_;
  Vector2i image_size_rgb_, image_size_depth_;
  sensor_msgs::CameraInfo rgb_info_;
  sensor_msgs::CameraInfo depth_info_;

  tf::TransformListener listener;
  tf::StampedTransform tf_world_to_camera_transform_current_;

  //! Name for the depth camera frame id in TF.
  std::string camera_frame_id_;
  //! Name for the depth camera frame id in TF at the beginning.
  std::string camera_initial_frame_id_;
  //! Name for the fixed frame in TF.
  std::string world_frame_id_;
  /*!
   * Translation Vector, represents translation from the world origin to the
   * camera pose.
   */
  Vector3f infinitam_translation_vector_;
  /*!
   * Rotation Matrix, represents rotation from the world origin to the
   * camera pose.
   */
  Matrix3f infinitam_rotation_matrix_;
  //! True if the the camera pose is to be set from the incoming tf messages.
  bool set_camera_pose_;
  //! True if one has got tf message.
  bool got_tf_msg_;

 public:
  RosImageSourceEngine(ros::NodeHandle& nh, const char*& calibration_filename);

  ~RosImageSourceEngine();
  void rgbCallback(const sensor_msgs::Image::ConstPtr& msg);
  void rgbCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void depthCallback(const sensor_msgs::Image::ConstPtr& msg);
  void depthCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void setCameraPoseFromTF(const ros::Time& depth_img_time_stamp);

  // ImageSourceEngine
  bool hasMoreImages(void);
  void getImages(ITMUChar4Image* rgb, ITMShortImage* raw_depth);
  Vector2i getDepthImageSize(void);
  Vector2i getRGBImageSize(void);
};
}  // namespace Engine
}  // namespace InfiniTAM
