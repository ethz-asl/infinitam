// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "PoseSourceEngine.h"

#include <glog/logging.h>
#include <cstdio>
#include <mutex>
#include <string>

#include "../ITMLib/Utils/ITMLibDefines.h"

#if (!defined USING_CMAKE) && (defined _MSC_VER)
#ifdef _DEBUG
#pragma comment(lib, "libpxcmd_d")
#else
#pragma comment(lib, "libpxcmd")
#endif
#endif
#ifdef COMPILE_WITH_Ros

#include <ros/ros.h>
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

  //! Name for the fixed frame in TF.
  std::string world_frame_id_;

  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;

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

  //! TF transforms between camera and world frame.
  tf::StampedTransform tf_world_to_camera_transform_current_;
  tf::StampedTransform tf_world_to_camera_transform_at_start_;

 public:
  //! True if the the camera pose is to be set from the incoming tf messages.
  bool set_camera_pose_;

  //! True if one has got tf message.
  bool got_tf_msg_;

  /*!
   * Constructor
   */
  RosPoseSourceEngine(ros::NodeHandle& nh);

  /*!
   * Destructor
   */
  ~RosPoseSourceEngine();

  /*!
   * Callback for the incoming TF messages. This method is called by the ROS
   * nodehandle after a TF message has arrived.
   */
  void TFCallback(const tf::tfMessage& tf_msg);

  bool hasMoreMeasurements(void);
  void getMeasurement(ITMPoseMeasurement* pose);
};
}  // namespace Engine
}  // namespace InfiniTAM
