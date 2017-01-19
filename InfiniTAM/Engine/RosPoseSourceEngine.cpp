// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "RosPoseSourceEngine.h"

#include <stdexcept>
#include <string>

#ifdef COMPILE_WITH_Ros

namespace InfiniTAM {
namespace Engine {

RosPoseSourceEngine::RosPoseSourceEngine(ros::NodeHandle& nh)
    : PoseSourceEngine(),
      first_time_tf_available_(true),
      broadcast_transformations(false),
      got_tf_msg_(false),
      set_camera_pose_(true) {
  // Set the topic and frame_id names.
  nh.param<std::string>("camera_frame_id", camera_frame_id_,
                        "camera_depth_optical_frame");
  nh.param<std::string>("camera_initial_frame_id", camera_initial_frame_id_,
                        "tf_camera_initial");
  nh.param<std::string>("world_frame_id", world_frame_id_, "world");
}

RosPoseSourceEngine::~RosPoseSourceEngine() {}

// Get the pose of the camera from the forward kinematics of the robot.
void RosPoseSourceEngine::TFCallback(const tf::tfMessage& tf_msg) {
  try {
    if (first_time_tf_available_) {
      // Check for the first time that the main_engine_ pointer is not null.
      CHECK_NOTNULL(main_engine_);
      // If one is streaming over rosbag, give it some time to send the
      // transform.
      listener.waitForTransform(camera_frame_id_, world_frame_id_, ros::Time(0),
                                ros::Duration(5.0));
      first_time_tf_available_time_ = ros::Time::now();
    }

    // Get the pose of the camera from the same time stamp as the latest image.
    // Images messages have a lower frequency then pose messages.
    latest_depth_image_stamp_.fromSec(main_engine_->getImageTimeStamp());
    // If first image has not arrived yet,
    if (first_time_tf_available_time_ > latest_depth_image_stamp_) {
      listener.lookupTransform(camera_frame_id_, world_frame_id_, ros::Time(0),
                               tf_world_to_camera_transform_current_);
      ROS_WARN("Getting the latest transform and not the image-synced one ");
    } else {
      listener.waitForTransform(camera_frame_id_, world_frame_id_,
                                latest_depth_image_stamp_, ros::Duration(0.5));
      listener.lookupTransform(camera_frame_id_, world_frame_id_,
                               latest_depth_image_stamp_,
                               tf_world_to_camera_transform_current_);
    }

    // Get the first transformation between the world and the camera.
    if (first_time_tf_available_) {
      got_tf_msg_ = true;
      tf_world_to_camera_transform_at_start_ =
          tf_world_to_camera_transform_current_;

      // Origin of the map for infinitam.
      broadcaster.sendTransform(tf::StampedTransform(
          tf_world_to_camera_transform_at_start_, ros::Time::now(),
          world_frame_id_, camera_initial_frame_id_));
      ROS_INFO("Got first TF message.");
      first_time_tf_available_ = false;
      broadcast_transformations = true;
    }

    infinitam_translation_vector_.x =
        tf_world_to_camera_transform_current_.getOrigin().getX();
    infinitam_translation_vector_.y =
        tf_world_to_camera_transform_current_.getOrigin().getY();
    infinitam_translation_vector_.z =
        tf_world_to_camera_transform_current_.getOrigin().getZ();

    // Invert the rotation since we use this in the perspective of the camera.
    tf_world_to_camera_transform_current_.setBasis(
        tf_world_to_camera_transform_current_.getBasis().inverse());

    infinitam_rotation_matrix_.m00 =
        tf_world_to_camera_transform_current_.getBasis().getColumn(0).getX();
    infinitam_rotation_matrix_.m10 =
        tf_world_to_camera_transform_current_.getBasis().getColumn(0).getY();
    infinitam_rotation_matrix_.m20 =
        tf_world_to_camera_transform_current_.getBasis().getColumn(0).getZ();
    infinitam_rotation_matrix_.m01 =
        tf_world_to_camera_transform_current_.getBasis().getColumn(1).getX();
    infinitam_rotation_matrix_.m11 =
        tf_world_to_camera_transform_current_.getBasis().getColumn(1).getY();
    infinitam_rotation_matrix_.m21 =
        tf_world_to_camera_transform_current_.getBasis().getColumn(1).getZ();
    infinitam_rotation_matrix_.m02 =
        tf_world_to_camera_transform_current_.getBasis().getColumn(2).getX();
    infinitam_rotation_matrix_.m12 =
        tf_world_to_camera_transform_current_.getBasis().getColumn(2).getY();
    infinitam_rotation_matrix_.m22 =
        tf_world_to_camera_transform_current_.getBasis().getColumn(2).getZ();

    if (broadcast_transformations) {
      // Visualize the transformations as ROS tfs.

      // Broadcast the tf for initianl camera pose.
      broadcaster.sendTransform(tf::StampedTransform(
          tf_world_to_camera_transform_at_start_, ros::Time::now(),
          world_frame_id_, camera_initial_frame_id_));
    }

    if (set_camera_pose_) {
      // Assign the infinitam camera pose the same pose as TF.
      main_engine_->GetTrackingState()->pose_d->SetT(
          infinitam_translation_vector_);
      main_engine_->GetTrackingState()->pose_d->SetR(
          infinitam_rotation_matrix_);
    }
  } catch (tf::TransformException& ex) {
    ROS_ERROR("Exception catched: %s", ex.what());
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
