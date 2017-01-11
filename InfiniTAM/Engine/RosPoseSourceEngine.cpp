// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "RosPoseSourceEngine.h"

#include <stdexcept>
#include <string>

#include "../Utils/FileUtils.h"

#ifdef COMPILE_WITH_Ros

namespace InfiniTAM {
namespace Engine {

RosPoseSourceEngine::RosPoseSourceEngine(ros::NodeHandle& nh)
    : PoseSourceEngine(),
      first_time_tf_available_(true),
      broadcast_transformations(false) {
  set_camera_pose_ = true;
  // set the topic and frame_id names.
  nh.param<std::string>("camera_frame_id", camera_frame_id_,
                        "camera_depth_optical_frame");
  nh.param<std::string>("world_frame_id", world_frame_id_, "world");
  nh.param<std::string>("complete_cloud", complete_cloud_topic_,
                        "/complete_cloud");
}

RosPoseSourceEngine::~RosPoseSourceEngine() {}

// Get the pose of the camera from the forward kinematics of the robot.
void RosPoseSourceEngine::TFCallback(const tf::tfMessage& tf_msg) {
  try {
    CHECK_NOTNULL(main_engine_);

    // Get the pose of the camera from the same time stamp as the latest image.
    // Images messages have a lower frequency then pose messages.
//    latest_depth_image_stamp_.fromSec(main_engine_->getImageTimeStamp());
    latest_depth_image_stamp_=ros::Time(0);
    // Get transform from world to camera frame.
    listener.lookupTransform(world_frame_id_, camera_frame_id_,
                             latest_depth_image_stamp_,
                             tf_world_to_camera_transform_current_);

    // Get the first transformation between the world and the camera.
    if (first_time_tf_available_) {
      tf_world_to_camera_transform_at_start_ =
          tf_world_to_camera_transform_current_;
      // Set the infinitam inertial coordinate system.
      Vector3f zero_vector;
      zero_vector.x = zero_vector.y = zero_vector.z = 0.0f;
      Matrix3f identity;
      identity.setIdentity();
      main_engine_->GetTrackingState()->pose_d->SetT(zero_vector);
      main_engine_->GetTrackingState()->pose_d->SetR(identity);

      // origin of the map for infinitam
      broadcaster.sendTransform(
          tf::StampedTransform(tf_world_to_camera_transform_at_start_,
                               ros::Time::now(), "world", "tf_camera_initial"));
      ROS_INFO("Got first TF message.");
      first_time_tf_available_ = false;
      broadcast_transformations = false;
    }

    // Get Infinitam origin to current camera frame transform.
    tf_infinitam_origin_to_camera_transform_relative_.frame_id_ =
        "tf_camera_initial";
    tf_infinitam_origin_to_camera_transform_relative_.stamp_ =
        latest_depth_image_stamp_;

    tf_infinitam_origin_to_camera_current_rotation =
        tf_world_to_camera_transform_at_start_.getBasis().inverse() *
        tf_world_to_camera_transform_current_.getBasis();

    tf_infinitam_origin_to_camera_current_translation_in_world_frame =
        -tf_world_to_camera_transform_at_start_.getOrigin() +
        tf_world_to_camera_transform_current_.getOrigin();

    tf_infinitam_origin_to_camera_current_translation_in_infinitam_frame =
        tf_world_to_camera_transform_at_start_.getBasis() *
        tf_infinitam_origin_to_camera_current_translation_in_world_frame;

    // Camera transform wrt infinitam origin in RVIZ.
    tf_infinitam_origin_to_camera_transform_relative_.setOrigin(
        tf_infinitam_origin_to_camera_current_translation_in_infinitam_frame);
    tf_infinitam_origin_to_camera_transform_relative_.setBasis(
        tf_infinitam_origin_to_camera_current_rotation);

    tf_infinitam_origin_to_camera_current_translation_in_infinitam_frame_rotated =
        tf_infinitam_origin_to_camera_current_rotation.inverse() *
        tf_infinitam_origin_to_camera_current_translation_in_infinitam_frame;

    infinitam_translation_vector_.x =
        -tf_infinitam_origin_to_camera_current_translation_in_infinitam_frame_rotated
             .getX();
    infinitam_translation_vector_.y =
        -tf_infinitam_origin_to_camera_current_translation_in_infinitam_frame_rotated
             .getY();
    infinitam_translation_vector_.z =
        -tf_infinitam_origin_to_camera_current_translation_in_infinitam_frame_rotated
             .getZ();

    infinitam_rotation_matrix_.m00 =
        tf_infinitam_origin_to_camera_transform_relative_.getBasis()
            .getColumn(0)
            .getX();
    infinitam_rotation_matrix_.m10 =
        tf_infinitam_origin_to_camera_transform_relative_.getBasis()
            .getColumn(0)
            .getY();
    infinitam_rotation_matrix_.m20 =
        tf_infinitam_origin_to_camera_transform_relative_.getBasis()
            .getColumn(0)
            .getZ();
    infinitam_rotation_matrix_.m01 =
        tf_infinitam_origin_to_camera_transform_relative_.getBasis()
            .getColumn(1)
            .getX();
    infinitam_rotation_matrix_.m11 =
        tf_infinitam_origin_to_camera_transform_relative_.getBasis()
            .getColumn(1)
            .getY();
    infinitam_rotation_matrix_.m21 =
        tf_infinitam_origin_to_camera_transform_relative_.getBasis()
            .getColumn(1)
            .getZ();
    infinitam_rotation_matrix_.m02 =
        tf_infinitam_origin_to_camera_transform_relative_.getBasis()
            .getColumn(2)
            .getX();
    infinitam_rotation_matrix_.m12 =
        tf_infinitam_origin_to_camera_transform_relative_.getBasis()
            .getColumn(2)
            .getY();
    infinitam_rotation_matrix_.m22 =
        tf_infinitam_origin_to_camera_transform_relative_.getBasis()
            .getColumn(2)
            .getZ();

    ROS_DEBUG_STREAM("\n transformation infinitam origin to camera pose: \n"
                     << infinitam_translation_vector_ << "\n");
    ROS_DEBUG_STREAM("\n transformation infinitam origin to camera pose: \n"
                     << infinitam_rotation_matrix_ << "\n");

    if (broadcast_transformations) {
      // Visualize the transformations as ROS tfs.

      // Broadcast the tf for origin of the map for infinitam.
      broadcaster.sendTransform(
          tf::StampedTransform(tf_world_to_camera_transform_at_start_,
                               ros::Time(0), "world", "tf_camera_initial"));
      // Broadcast the tf for the current pose of the camera in infinitam
      // coordinate system.
      broadcaster.sendTransform(tf::StampedTransform(
          tf_infinitam_origin_to_camera_transform_relative_, ros::Time(0),
          "tf_camera_initial", "infinitam_pose"));
    }

    if (set_camera_pose_) {
      // Assign the infinitam camera pose the same pose as TF.
      main_engine_->GetTrackingState()->pose_d->SetT(
          infinitam_translation_vector_);
      main_engine_->GetTrackingState()->pose_d->SetR(
          infinitam_rotation_matrix_);
    }
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
  }
}

bool RosPoseSourceEngine::hasMoreMeasurements(void) {
  return (cached_pose != NULL);
}

void RosPoseSourceEngine::getMeasurement(ITMPoseMeasurement* pose) {
  if (cached_pose != NULL) {
    ROS_INFO("Using Pose data...");
    pose->R = cached_pose->R;
    pose->T = cached_pose->T;
    delete cached_pose;
    cached_pose = NULL;
  }
}

}  // namespace Engine
}  // namespace InfiniTAM
#else

namespace InfiniTAM {
namespace Engine {

RosPoseSourceEngine::RosPoseSourceEngine(const ros::NodeHandle& nh)
    : PoseSourceEngine() {
  printf("Compiled without ROS support.\n");
}
RosPoseSourceEngine::~RosPoseSourceEngine() {}
}  // namespace Engine
}  // namespace InfiniTAM

#endif
