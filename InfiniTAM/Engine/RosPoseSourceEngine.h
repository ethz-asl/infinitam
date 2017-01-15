// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <glog/logging.h>
#include <cstdio>
#include <mutex>
#include <string>

#include "../ITMLib/Utils/ITMLibDefines.h"
#include "PoseSourceEngine.h"

#if (!defined USING_CMAKE) && (defined _MSC_VER)
#ifdef _DEBUG
#pragma comment(lib, "libpxcmd_d")
#else
#pragma comment(lib, "libpxcmd")
#endif
#endif
#ifdef COMPILE_WITH_Ros
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
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#endif

namespace InfiniTAM {
namespace Engine {
class RosPoseSourceEngine : public PoseSourceEngine {
 private:
  //! True if one wants to broadcast the transforms calculated here.
  bool broadcast_transformations;

  //! Is set to false after first tf arrives, this is used to store the first
  //! transformation in a separate variable.
  bool first_time_tf_available_;
  ros::Time first_time_tf_available_time_;

  //! Time stamp of the latest available image.
  ros::Time latest_depth_image_stamp_;

  //! Name for the depth camera frame id in TF.
  std::string camera_frame_id_;
  //! Name for the depth camera frame id in TF at the beginning.
  std::string camera_initial_frame_id_;
  //! Only for testing.
  std::string current_infinitam_frame_id_;

  //! Name for the fixed frame in TF.
  std::string world_frame_id_;
  //! ROS topic name where the generated complete cloud is published.
  std::string complete_cloud_topic_;

  //! ROS service name, when called the current mesh is transformed into a point
  //! cloud and published.
  ros::ServiceServer publish_scene_service_;
  //! ROS publisher to send out the complete cloud.
  ros::Publisher complete_point_cloud_pub_;

  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;

  // Infinitam Vector, represents translation from the infinitam origin to the
  // camera pose.
  Vector3f infinitam_translation_vector_;
  // Infinitam Matrix, represents rotation from the infinitam origin to the
  // camera pose.
  Matrix3f infinitam_rotation_matrix_;

 public:
  //! Set set the camera pose from the incoming tf messages.
  bool set_camera_pose_;

  //! True if one has got tf
  bool got_tf_msg_;

  tf::StampedTransform tf_world_to_camera_transform_current_;
  tf::StampedTransform world_to_camera_transform_current_test_;
  tf::StampedTransform tf_world_to_camera_transform_at_start_;
  tf::StampedTransform tf_infinitam_origin_to_camera_transform_relative_;
  tf::Matrix3x3 tf_infinitam_origin_to_camera_current_rotation;
  tf::Vector3 tf_infinitam_origin_to_camera_current_translation_in_world_frame,
      tf_infinitam_origin_to_camera_current_translation_in_infinitam_frame,
      tf_infinitam_origin_to_camera_current_translation_in_infinitam_frame_rotated;

  RosPoseSourceEngine(ros::NodeHandle& nh);
  ~RosPoseSourceEngine();

  void TFCallback(const tf::tfMessage& tf_msg);

  bool hasMoreMeasurements(void);
  void getMeasurement(ITMPoseMeasurement* pose);
};
}  // namespace Engine
}  // namespace InfiniTAM
