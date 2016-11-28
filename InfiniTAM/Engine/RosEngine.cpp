// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "RosEngine.h"

#include "../Utils/FileUtils.h"

#include <cstdio>
#include <stdexcept>

#ifdef COMPILE_WITH_Ros

using namespace InfiniTAM::Engine;

RosEngine::RosEngine(ros::NodeHandle& nh, const char*& calibration_filename)
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

  depth_info_sub =
      nh.subscribe(depth_camera_info_topic_, 1,
                   &RosEngine::depthCameraInfoCallback, (RosEngine*)this);
  rgb_info_sub =
      nh.subscribe(rgb_camera_info_topic_, 1, &RosEngine::rgbCameraInfoCallback,
                   (RosEngine*)this);

  complete_point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(
      "/complete_table_top_scene_topic", 1);

  while (!rgb_info_ready_ || !depth_info_ready_) {
    ROS_INFO("Spinning, waiting for rgb and depth camera info messages.");
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  // initialize service
  publish_scene_service_ = nh.advertiseService(
      "publish_scene", &RosEngine::PublishMap, (RosEngine*)this);

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

RosEngine::~RosEngine() {}

void RosEngine::rgbCallback(const sensor_msgs::Image::ConstPtr& msg) {
  if (!rgb_ready_ && data_available_) {
    std::lock_guard<std::mutex> guard(rgb_mutex_);
    rgb_ready_ = true;

    cv_rgb_image_ =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  }
}

void RosEngine::depthCallback(const sensor_msgs::Image::ConstPtr& msg) {
  if (!depth_ready_ && data_available_) {
    std::lock_guard<std::mutex> guard(depth_mutex_);
    depth_ready_ = true;

    cv_depth_image_ =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
  }
}

void RosEngine::rgbCameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& msg) {
  image_size_rgb_.x = msg->width;
  image_size_rgb_.y = msg->height;
  rgb_info_ = *msg;
  rgb_info_ready_ = true;
  ROS_INFO("Got rgb camera info.");
}

void RosEngine::depthCameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& msg) {
  image_size_depth_.x = msg->width;
  image_size_depth_.y = msg->height;
  depth_info_ = *msg;
  depth_info_ready_ = true;
  ROS_INFO("Got depth camera info.");
}

void RosEngine::getImages(ITMUChar4Image* rgb_image,
                          ITMShortImage* raw_depth_image) {
  // ROS_INFO("getImages().");
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
  // ROS_INFO("processing rgb.");
  for (size_t i = 0; i < 3 * rgb_rows * rgb_cols; i += 3) {
    Vector4u pixel_value;
    pixel_value.r = cv_rgb_image_->image.data[i];
    pixel_value.g = cv_rgb_image_->image.data[i + 1];
    pixel_value.b = cv_rgb_image_->image.data[i + 2];
    pixel_value.w = 255;
    rgb_infinitam[i / 3] = pixel_value;
  }
  // ROS_INFO("processing rgb done.");
  rgb_ready_ = false;
  depth_ready_ = false;
  data_available_ = true;
}

// bool RosEngine::hasMoreImages(void) { return (rgb_ready_ && depth_ready_); }
bool RosEngine::hasMoreImages(void) {
  if (!rgb_ready_ || !depth_ready_) {
    ros::spinOnce();
    return false;
  }
  return true;
}
Vector2i RosEngine::getDepthImageSize(void) { return image_size_depth_; }
Vector2i RosEngine::getRGBImageSize(void) { return image_size_rgb_; }

sensor_msgs::PointCloud2 RosEngine::conversionToPCL() {
  ROS_INFO("conversionToPCL() start");
  pcl::PointCloud<pcl::PointXYZ> complete_point_cloud;
  sensor_msgs::PointCloud2 complete_point_cloud2;

  if (main_engine_ != NULL) {

    CHECK_NOTNULL(main_engine_->GetMesh());

    main_engine_->GetMeshingEngine()->MeshScene(main_engine_->GetMesh(),
                                           main_engine_->GetScene());

    ORUtils::MemoryBlock<ITMMesh::Triangle>* cpu_triangles;
    bool shoulDelete = false;

    if (main_engine_->GetMesh()->memoryType == MEMORYDEVICE_CUDA) {
      cpu_triangles = new ORUtils::MemoryBlock<ITMMesh::Triangle>(
          main_engine_->GetMesh()->noMaxTriangles, MEMORYDEVICE_CPU);
      cpu_triangles->SetFrom(
          main_engine_->GetMesh()->triangles,
          ORUtils::MemoryBlock<ITMMesh::Triangle>::CUDA_TO_CPU);
      shoulDelete = true;
    } else {
      cpu_triangles = main_engine_->GetMesh()->triangles;
    }

    ITMMesh::Triangle* triangleArray = cpu_triangles->GetData(MEMORYDEVICE_CPU);

    complete_point_cloud.width =
        main_engine_->GetMesh()->noTotalTriangles * 3; // Point cloud has 3 points per triangle.
    complete_point_cloud.height =
        1;  // TODO(gocarlos) how to convert to structured point cloud?
    complete_point_cloud.is_dense = false;
    complete_point_cloud.points.resize(complete_point_cloud.width *
                                       complete_point_cloud.height);

    ROS_INFO_STREAM("this_mesh->noTotalTriangles: "
                    << main_engine_->GetMesh()->noTotalTriangles);

    for (long i = 0; i < main_engine_->GetMesh()->noTotalTriangles * 3;
         i = i + 3) {
      complete_point_cloud.points[i].x = triangleArray[i].p0.x;
      complete_point_cloud.points[i].y = triangleArray[i].p0.y;
      complete_point_cloud.points[i].z = triangleArray[i].p0.z;

      complete_point_cloud.points[i + 1].x = triangleArray[i].p1.x;
      complete_point_cloud.points[i + 1].y = triangleArray[i].p1.y;
      complete_point_cloud.points[i + 1].z = triangleArray[i].p1.z;

      complete_point_cloud.points[i + 2].x = triangleArray[i].p2.x;
      complete_point_cloud.points[i + 2].y = triangleArray[i].p2.y;
      complete_point_cloud.points[i + 2].z = triangleArray[i].p2.z;

    }

    if (shoulDelete) {
      delete cpu_triangles;
    }

    pcl::toROSMsg(complete_point_cloud, complete_point_cloud2);
    complete_point_cloud2.header.frame_id = "sr300_depth_frame";
    return complete_point_cloud2;

  } else {
    ROS_ERROR("main_engine_ is NULL");
  }
  return complete_point_cloud2;
}

bool RosEngine::PublishMap(std_srvs::Empty::Request& request,
                           std_srvs::Empty::Response& response) {

  complete_point_cloud_pub_.publish(conversionToPCL());

  return true;
}

#else

using namespace InfiniTAM::Engine;

RosEngine::RosEngine(const ros::NodeHandle& nh,
                     const char*& calibration_filename)
    : ImageSourceEngine(calibration_filename) {
  printf("Compiled without ROS support.\n");
}
RosEngine::~RosEngine() {}
void RosEngine::getImages(ITMUChar4Image* rgb_image,
                          ITMShortImage* raw_depth_image) {
  return;
}
bool RosEngine::hasMoreImages(void) { return false; }
Vector2i RosEngine::getDepthImageSize(void) { return Vector2i(0, 0); }
Vector2i RosEngine::getRGBImageSize(void) { return Vector2i(0, 0); }

#endif
