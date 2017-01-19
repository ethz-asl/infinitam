// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "RosImageSourceEngine.h"

#include <cstdio>
#include <stdexcept>
#include <string>

#include "../Utils/FileUtils.h"

#ifdef COMPILE_WITH_Ros

namespace InfiniTAM {
namespace Engine {

RosImageSourceEngine::RosImageSourceEngine(ros::NodeHandle& nh,
                                           const char*& calibration_filename)
    : ImageSourceEngine(calibration_filename),
      rgb_ready_(false),
      depth_ready_(false),
      rgb_info_ready_(false),
      depth_info_ready_(false),
      data_available_(true) {
  ros::Subscriber rgb_info_sub;
  ros::Subscriber depth_info_sub;

  nh.param<std::string>("rgb_camera_info_topic", rgb_camera_info_topic_,
                        "/camera/rgb/camera_info");
  nh.param<std::string>("depth_camera_info_topic", depth_camera_info_topic_,
                        "/camera/depth/camera_info");

  depth_info_sub = nh.subscribe(depth_camera_info_topic_, 1,
                                &RosImageSourceEngine::depthCameraInfoCallback,
                                static_cast<RosImageSourceEngine*>(this));
  rgb_info_sub = nh.subscribe(rgb_camera_info_topic_, 1,
                              &RosImageSourceEngine::rgbCameraInfoCallback,
                              static_cast<RosImageSourceEngine*>(this));

  while (!rgb_info_ready_ || !depth_info_ready_) {
    ROS_INFO("Spinning, waiting for rgb and depth camera info messages.");
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  // ROS depth images come in millimeters... (or in floats, which we don't
  // support yet)
  this->calib.disparityCalib.type = ITMDisparityCalib::TRAFO_AFFINE;
  this->calib.disparityCalib.params = Vector2f(1.0f / 1000.0f, 0.0f);

  this->calib.trafo_rgb_to_depth = ITMExtrinsics();
  this->calib.intrinsics_d = this->calib.intrinsics_rgb;

  this->calib.intrinsics_d.SetFrom(depth_info_.K[0], depth_info_.K[4],
                                   depth_info_.K[2], depth_info_.K[5],
                                   image_size_depth_.x, image_size_depth_.y);
  this->calib.intrinsics_rgb.SetFrom(rgb_info_.K[0], rgb_info_.K[4],
                                     rgb_info_.K[2], rgb_info_.K[5],
                                     image_size_rgb_.x, image_size_rgb_.y);
  ROS_INFO("RGB camera intrinsics: %f, %f, %f, %f, %d, %d", rgb_info_.K[0],
           rgb_info_.K[4], rgb_info_.K[2], rgb_info_.K[5], image_size_rgb_.x,
           image_size_rgb_.y);
  ROS_INFO("Depth camera intrinsics: %f, %f, %f, %f, %d, %d", depth_info_.K[0],
           depth_info_.K[4], depth_info_.K[2], depth_info_.K[5],
           image_size_depth_.x, image_size_depth_.y);
  this->calib.disparityCalib.params = Vector2f(1.0f / 1000.0f, 0.0f);
}

RosImageSourceEngine::~RosImageSourceEngine() {
}

void RosImageSourceEngine::rgbCallback(
    const sensor_msgs::Image::ConstPtr& msg) {
  ROS_INFO_ONCE("Got rgb raw image.");
  if (!rgb_ready_ && data_available_) {
    std::lock_guard<std::mutex> guard(rgb_mutex_);
    rgb_ready_ = true;

    cv_rgb_image_ = cv_bridge::toCvCopy(msg,
                                        sensor_msgs::image_encodings::RGB8);
  }
}

void RosImageSourceEngine::depthCallback(
    const sensor_msgs::Image::ConstPtr& msg) {
  ROS_INFO_ONCE("Got depth raw image.");
  if (!depth_ready_ && data_available_) {
    std::lock_guard<std::mutex> guard(depth_mutex_);
    depth_ready_ = true;
    depth_msg_time_stamp_ = msg->header.stamp;
    main_engine_->setImageTimeStamp(depth_msg_time_stamp_.toSec());
    CHECK(
        msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1
            || msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1);
    cv_depth_image_ = cv_bridge::toCvCopy(msg, msg->encoding);
    // When streaming raw images from Gazebo.
    if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      constexpr double kDepthScalingFactor = 1000.0;
      // When doing live streaming from the camera.
      (cv_depth_image_->image)
          .convertTo(cv_depth_image_->image, CV_16UC1, kDepthScalingFactor);
    }
  }
}

void RosImageSourceEngine::rgbCameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& msg) {
  ROS_INFO("Got rgb camera info.");
  image_size_rgb_.x = msg->width;
  image_size_rgb_.y = msg->height;
  rgb_info_ = *msg;
  rgb_info_ready_ = true;
}

void RosImageSourceEngine::depthCameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& msg) {
  ROS_INFO("Got depth camera info.");
  image_size_depth_.x = msg->width;
  image_size_depth_.y = msg->height;
  depth_info_ = *msg;
  depth_info_ready_ = true;
}

void RosImageSourceEngine::getImages(ITMUChar4Image* rgb_image,
ITMShortImage* raw_depth_image) {
  // Wait for frames.
  if (!data_available_) {
    return;
  }
  data_available_ = false;

  // Setup infinitam depth frame.
  std::lock_guard<std::mutex> depth_guard(depth_mutex_);
  short* raw_depth_infinitam = raw_depth_image->GetData(MEMORYDEVICE_CPU);

  cv::Size depth_size = cv_depth_image_->image.size();
  uint depth_rows = depth_size.height;
  uint depth_cols = depth_size.width;
  for (size_t i = 0; i < depth_rows * depth_cols; ++i) {
    raw_depth_infinitam[i] =
    ((cv_depth_image_->image.data[2 * i + 1] << 8) & 0xFF00) |
    (cv_depth_image_->image.data[2 * i] & 0xFF);
  }

  // Setup infinitam rgb frame.
  std::lock_guard<std::mutex> rgb_guard(rgb_mutex_);
  Vector4u* rgb_infinitam = rgb_image->GetData(MEMORYDEVICE_CPU);

  cv::Size rgb_size = cv_rgb_image_->image.size();
  uint rgb_rows = rgb_size.height;
  uint rgb_cols = rgb_size.width;
  for (size_t i = 0; i < 3 * rgb_rows * rgb_cols; i += 3) {
    Vector4u pixel_value;
    pixel_value.r = cv_rgb_image_->image.data[i];
    pixel_value.g = cv_rgb_image_->image.data[i + 1];
    pixel_value.b = cv_rgb_image_->image.data[i + 2];
    pixel_value.w = 255;
    rgb_infinitam[i / 3] = pixel_value;
  }

  rgb_ready_ = false;
  depth_ready_ = false;
  data_available_ = true;
}

bool RosImageSourceEngine::hasMoreImages(void) {
  if (!rgb_ready_ || !depth_ready_) {
    ros::spinOnce();
    return false;
  }
  return true;
}
Vector2i RosImageSourceEngine::getDepthImageSize(void) {
  return image_size_depth_;
}
Vector2i RosImageSourceEngine::getRGBImageSize(void) {
  return image_size_rgb_;
}

}  // namespace Engine
}  // namespace InfiniTAM
#else

namespace InfiniTAM {
  namespace Engine {

    RosImageSourceEngine::RosImageSourceEngine(const ros::NodeHandle& nh,
        const char*& calibration_filename)
    : ImageSourceEngine(calibration_filename) {
      printf("Compiled without ROS support.\n");
    }
    RosImageSourceEngine::~RosImageSourceEngine() {}
    void RosImageSourceEngine::getImages(ITMUChar4Image* rgb_image,
        ITMShortImage* raw_depth_image) {
      return;
    }
    bool RosImageSourceEngine::hasMoreImages(void) {return false;}
    Vector2i RosImageSourceEngine::getDepthImageSize(void) {
      return Vector2i(0, 0);
    }
    Vector2i RosImageSourceEngine::getRGBImageSize(void) {return Vector2i(0, 0);}
  }  // namespace Engine
}  // namespace InfiniTAM

#endif
