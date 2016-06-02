// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "RosEngine.h"

#include "../Utils/FileUtils.h"

#include <cstdio>
#include <stdexcept>

#ifdef COMPILE_WITH_Ros

using namespace InfiniTAM::Engine;

class RosEngine::PrivateData {
public:
  PrivateData(void) {}
  ros::Publisher point_cloud_pub;
  // rs::context ctx;
};

RosEngine::RosEngine(ros::NodeHandle *nh, const char *calibFilename,
                     Vector2i requested_imageSize_rgb,
                     Vector2i requested_imageSize_d)
    : ImageSourceEngine(calibFilename), rgb_ready_(false), depth_ready_(false),
      rgb_info_ready_(false), depth_info_ready_(false), dataAvailable(true) {
  // ROS_INFO("Constructor RosEngine.");
  ros::Subscriber rgb_info_sub;
  ros::Subscriber depth_info_sub;
  // rgb_info_sub_ = node_handle->subscribe("/camera/rgb/camera_info", 1,
  //                                     &RosEngine::rgbCameraInfoCallback,
  //                                     (RosEngine *)imageSource);

  // TODO(ff): Read topics from params.
  depth_info_sub =
      nh->subscribe("/camera/depth/camera_info", 1,
                    &RosEngine::depthCameraInfoCallback, (RosEngine *)this);
  rgb_info_sub =
      nh->subscribe("/camera/color/camera_info", 1,
                    &RosEngine::rgbCameraInfoCallback, (RosEngine *)this);
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

  // this->imageSize_depth = requested_imageSize_d;
  // this->imageSize_rgb = requested_imageSize_rgb;
  depth_frame_ = new unsigned char[imageSize_depth.x * imageSize_depth.y * 2];
  rgb_frame_ = new unsigned char[imageSize_rgb.x * imageSize_rgb.y * 3];

  this->calib.intrinsics_d.SetFrom(depth_info_.K[0], depth_info_.K[4],
                                   depth_info_.K[2], depth_info_.K[5],
                                   imageSize_depth.x, imageSize_depth.y);
  this->calib.intrinsics_rgb.SetFrom(rgb_info_.K[0], rgb_info_.K[4],
                                     rgb_info_.K[2], rgb_info_.K[5],
                                     imageSize_rgb.x, imageSize_rgb.y);
  ROS_INFO("Camera intrinsics: %f, %f, %f, %f, %d, %d", rgb_info_.K[0],
           rgb_info_.K[4], rgb_info_.K[2], rgb_info_.K[5], imageSize_rgb.x,
           imageSize_rgb.y);
  this->calib.disparityCalib.params = Vector2f(1.0f / 1000.0f, 0.0f);
  //
  // data->dev->start();
}

RosEngine::~RosEngine() {
  // if (data != NULL) {
  //   data->dev->stop();
  //   delete data;
  // }
  delete[] depth_frame_;
  delete[] rgb_frame_;
}

void RosEngine::rgbCallback(const sensor_msgs::Image::ConstPtr &msg) {
  if (!rgb_ready_ && dataAvailable) {
    ROS_INFO("Got RGB image.");
    std::lock_guard<std::mutex> guard(rgb_mutex_);
    rgb_frame_ = msg->data.data();
    rgb_ready_ = true;
    ROS_INFO("Got RGB image end.");
  }
}

void RosEngine::depthCallback(const sensor_msgs::Image::ConstPtr &msg) {
  if (!depth_ready_ && dataAvailable) {
    ROS_INFO("Got depth image.");
    std::lock_guard<std::mutex> guard(depth_mutex_);
    depth_frame_ = msg->data.data();
    depth_ready_ = true;
    // ros::Duration(0.001).sleep();
    ROS_INFO("Got depth image end.");
  }
}

void RosEngine::rgbCameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr &msg) {
  imageSize_rgb.x = msg->width;
  imageSize_rgb.y = msg->height;
  rgb_info_ = *msg;
  rgb_info_ready_ = true;
  ROS_INFO("Got depth camera info.");
}

void RosEngine::depthCameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr &msg) {
  imageSize_depth.x = msg->width;
  imageSize_depth.y = msg->height;
  depth_info_ = *msg;
  depth_info_ready_ = true;
  ROS_INFO("Got depth camera info.");
}

void RosEngine::getImages(ITMUChar4Image *rgbImage,
                          ITMShortImage *rawDepthImage) {
  // ROS_INFO("getImages().");
  // ROS_INFO("depth_ready_: %s", depth_ready_ ? "true" : "false");
  // ROS_INFO("rgb_ready_: %s", rgb_ready_ ? "true" : "false");
  // Wait for frames.
  while (!rgb_ready_ || !depth_ready_) {
    // ROS_INFO("Spinning.");
    ros::spinOnce();
    ros::Duration(1.0 / 30.0).sleep();
  }
  dataAvailable = false;

  // Setup infinitam depth frame.
  std::lock_guard<std::mutex> depth_guard(depth_mutex_);
  short *rawDepth = rawDepthImage->GetData(MEMORYDEVICE_CPU);
  Vector2i depth_dims = rawDepthImage->noDims;
  rawDepthImage->Clear();
  ROS_INFO("processing depth.");
  for (int y = 0; y < depth_dims.y; y++) {
    for (int x = 0; x < depth_dims.x; x++) {
      unsigned int i = x + y * depth_dims.x;

      rawDepth[i] = ((depth_frame_[2 * i + 1] << 8) & 0xFF00) |
                    (depth_frame_[2 * i] & 0xFF);
    }
  }
  ROS_INFO("processing depth done.");
  // Setup infinitam rgb frame.
  std::lock_guard<std::mutex> rgb_guard(rgb_mutex_);
  Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
  rgbImage->Clear();
  ROS_INFO("processing rgb.");
  for (int i = 0; i < rgbImage->noDims.x * 3 * rgbImage->noDims.y; i += 3) {
    Vector4u newPix;
    newPix.r = rgb_frame_[i];
    newPix.g = rgb_frame_[i + 1];
    newPix.b = rgb_frame_[i + 2];
    newPix.w = 255;
    rgb[i / 3] = newPix;
  }
  ROS_INFO("processing rgb done.");
  rgb_ready_ = false;
  depth_ready_ = false;
  dataAvailable = true;
}

// bool RosEngine::hasMoreImages(void) { return (rgb_ready_ && depth_ready_); }
bool RosEngine::hasMoreImages(void) { return true; }
Vector2i RosEngine::getDepthImageSize(void) {
  // return (data != NULL) ? imageSize_depth : Vector2i(0, 0);
  return imageSize_depth;
}
Vector2i RosEngine::getRGBImageSize(void) {
  return imageSize_rgb;
  // return (data != NULL) ? imageSize_rgb : Vector2i(0, 0);
}

#else

using namespace InfiniTAM::Engine;

RosEngine::RosEngine(const char *calibFilename,
                     Vector2i requested_imageSize_rgb,
                     Vector2i requested_imageSize_d)
    : ImageSourceEngine(calibFilename) {
  printf("Compiled without ROS support.\n");
}
RosEngine::~RosEngine() {}
void RosEngine::getImages(ITMUChar4Image *rgbImage,
                          ITMShortImage *rawDepthImage) {
  return;
}
bool RosEngine::hasMoreImages(void) { return false; }
Vector2i RosEngine::getDepthImageSize(void) { return Vector2i(0, 0); }
Vector2i RosEngine::getRGBImageSize(void) { return Vector2i(0, 0); }

#endif
