// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ImageSourceEngine.h"

#include <mutex>

#if (!defined USING_CMAKE) && (defined _MSC_VER)
#ifdef _DEBUG
#pragma comment(lib, "libpxcmd_d")
#else
#pragma comment(lib, "libpxcmd")
#endif
#endif
// #ifdef COMPILE_WITH_Ros
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
// #endif

namespace InfiniTAM {
namespace Engine {
class RosEngine : public ImageSourceEngine {
private:
  class PrivateData;
  PrivateData *data;

  const unsigned char *rgb_frame_;
  const unsigned char *depth_frame_;
  bool rgb_ready_;
  bool depth_ready_;
  bool rgb_info_ready_;
  bool depth_info_ready_;
  bool dataAvailable;
  std::mutex rgb_mutex_;
  std::mutex depth_mutex_;
  Vector2i imageSize_rgb, imageSize_depth;
  sensor_msgs::CameraInfo rgb_info_;
  sensor_msgs::CameraInfo depth_info_;

public:
  RosEngine(ros::NodeHandle *nh, const char *calibFilename,
            Vector2i imageSize_rgb = Vector2i(640, 480),
            Vector2i imageSize_depth = Vector2i(640, 480));
  RosEngine(const char *calibFilename,
            Vector2i imageSize_rgb = Vector2i(640, 480),
            Vector2i imageSize_depth = Vector2i(640, 480));

  ~RosEngine();
  void rgbCallback(const sensor_msgs::Image::ConstPtr &msg);
  void rgbCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg);
  void depthCallback(const sensor_msgs::Image::ConstPtr &msg);
  void depthCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg);
  bool hasMoreImages(void);
  void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);
  Vector2i getDepthImageSize(void);
  Vector2i getRGBImageSize(void);
};
}
}
