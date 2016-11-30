// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "RosEngine.h"

#include <cstdio>
#include <stdexcept>
#include <string>

#include "../Utils/FileUtils.h"

#ifdef COMPILE_WITH_Ros

namespace InfiniTAM {
namespace Engine {

RosEngine::RosEngine(ros::NodeHandle& nh, const char*& calibration_filename)
    : ImageSourceEngine(calibration_filename),
      rgb_ready_(false),
      depth_ready_(false),
      rgb_info_ready_(false),
      tf_ready_(false),
      depth_info_ready_(false),
      data_available_(true) {
  ros::Subscriber rgb_info_sub;
  ros::Subscriber depth_info_sub;
  nh.param<std::string>("rgb_camera_info_topic", rgb_camera_info_topic_,
                        "/camera/rgb/camera_info");
  nh.param<std::string>("depth_camera_info_topic", depth_camera_info_topic_,
                        "/camera/depth/camera_info");

  nh.param<std::string>("camera_frame_id", camera_frame_id_,
                        "sr300_depth_frame");
  nh.param<std::string>("world_frame_id", world_frame_id_, "world");

  camera_pose_=new ITMPose;

  nh.param<std::string>("complete_table_top_scene_topic", complete_cloud_topic_,
                        "/complete_cloud");

  depth_info_sub = nh.subscribe(depth_camera_info_topic_, 1,
                                &RosEngine::depthCameraInfoCallback,
                                static_cast<RosEngine*>(this));
  rgb_info_sub =
      nh.subscribe(rgb_camera_info_topic_, 1, &RosEngine::rgbCameraInfoCallback,
                   static_cast<RosEngine*>(this));

  complete_point_cloud_pub_ =
      nh.advertise<sensor_msgs::PointCloud2>(complete_cloud_topic_, 1);

  marker_pub_ = nh.advertise<visualization_msgs::Marker>(
      "/visualization_marker", 200);

  while (!rgb_info_ready_ || !depth_info_ready_) {
    ROS_INFO("Spinning, waiting for rgb and depth camera info messages.");
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  ros::ServiceServer service = nh.advertiseService("service_name",
                                                   &RosEngine::PublishMap,
                                                   (RosEngine*) this);
  // initialize service
  publish_scene_service_ = nh.advertiseService(
      "publish_scene", &RosEngine::publishMap, static_cast<RosEngine*>(this));

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

RosEngine::~RosEngine() {
  delete camera_pose_;
}

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

// Get the pose of the camera from the forward kinematics of the robot.
void RosEngine::TFCallback(const tf::tfMessage &tf_msg) {
  ROS_INFO("TFCallback");

  // BUG: this is only done until "RGB camera intrinsics ..." after that no more calls.
  // some blocking thread?
  ROS_INFO("TFCallbacksadfasdf");

  ROS_INFO_STREAM("data_available_"<<data_available_);
  ROS_INFO_STREAM("tf_ready_"<<tf_ready_);
  ROS_INFO("wtf");
  if (!tf_ready_ && data_available_) {

//    std::lock_guard < std::mutex > guard(tf_mutex_);
    tf_ready_ = true;
    ROS_INFO("2");

//    int message_size = tf_msg.transforms.size();
//    ros::Time time = tf_msg.transforms[message_size].header.stamp;

    try {
      listener.lookupTransform(world_frame_id_, camera_frame_id_, ros::Time(0),
                               camera_base_transform_);
    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
//      ros::Duration(1.0).sleep();
    }
    ROS_INFO("3");

    std::string frame_id = camera_base_transform_.frame_id_;
    std::string frame2_id = camera_base_transform_.child_frame_id_;

    ROS_INFO_STREAM(
        "transform-> frame_id: " << frame_id << ", child_frame_id: " << frame2_id);

    double x, y, z;
    x = camera_base_transform_.getOrigin().getX();
    y = camera_base_transform_.getOrigin().getY();
    z = camera_base_transform_.getOrigin().getZ();

    double qx, qy, qz, qw;
    qx = camera_base_transform_.getRotation().getX();
    qy = camera_base_transform_.getRotation().getY();
    qz = camera_base_transform_.getRotation().getZ();
    qw = camera_base_transform_.getRotation().getW();

    double angle = 2 * acos(qw);
    double t = qx / sqrt(1 - qw * qw) * angle;
    double u = qy / sqrt(1 - qw * qw) * angle;
    double v = qz / sqrt(1 - qw * qw) * angle;

    camera_pose_->SetFrom(x, y, z, t, u, v);

    // just for testing --------------------------------------
    visualization_msgs::Marker camera_marker;
    camera_marker.header.frame_id = "world";
    camera_marker.header.stamp = ros::Time::now();

    camera_marker.ns = "camera_marker";
    camera_marker.id = 1;

    camera_marker.lifetime = ros::Duration();
    camera_marker.type = 3;
    camera_marker.action = visualization_msgs::Marker::ADD;

    camera_marker.pose.position.x = camera_base_transform_.getOrigin().x();
    camera_marker.pose.position.y = camera_base_transform_.getOrigin().y();
    camera_marker.pose.position.z = camera_base_transform_.getOrigin().z();

    camera_marker.pose.orientation.x = camera_base_transform_.getOrigin().x();
    camera_marker.pose.orientation.y = camera_base_transform_.getOrigin().y();
    camera_marker.pose.orientation.z = camera_base_transform_.getOrigin().z();
    camera_marker.pose.orientation.w = camera_base_transform_.getOrigin().w();

    camera_marker.color.r = ((double) rand() / (RAND_MAX));
    camera_marker.color.g = ((double) rand() / (RAND_MAX));
    camera_marker.color.b = ((double) rand() / (RAND_MAX));
    camera_marker.color.a = 0.8;

    camera_marker.scale.x = 0.05;
    camera_marker.scale.y = 0.05;
    camera_marker.scale.z = 0.1;
    marker_pub_.publish(camera_marker);
    // just for testing --------------------------------------
  }
ROS_INFO("asdfasdf");
}

void RosEngine::getMeasurement(ITMPose* pose) {
  ROS_INFO("getMeasurement().");
  if (!data_available_) {
    return;
  }
  std::lock_guard < std::mutex > tf_guard(tf_mutex_);
  tf_ready_ = false;

}

bool RosEngine::hasMoreMeasurements(void) {
  if (!tf_ready_) {
    ros::spinOnce();
    return false;
  }
  return true;
}

void RosEngine::getImages(ITMUChar4Image* rgb_image,
ITMShortImage* raw_depth_image) {
//  ROS_INFO("getImages().");

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

void RosEngine::extractMeshToPcl(
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud) {
  CHECK_NOTNULL(main_engine_);
  CHECK_NOTNULL(main_engine_->GetMesh());

  main_engine_->GetMeshingEngine()->MeshScene(main_engine_->GetMesh(),
                                              main_engine_->GetScene());

  ORUtils::MemoryBlock<ITMMesh::Triangle>* cpu_triangles;
  bool rm_triangle_in_cuda_memory = false;

  if (main_engine_->GetMesh()->memoryType == MEMORYDEVICE_CUDA) {
    cpu_triangles = new ORUtils::MemoryBlock<ITMMesh::Triangle>(
        main_engine_->GetMesh()->noMaxTriangles, MEMORYDEVICE_CPU);
    cpu_triangles->SetFrom(
        main_engine_->GetMesh()->triangles,
        ORUtils::MemoryBlock<ITMMesh::Triangle>::CUDA_TO_CPU);
    rm_triangle_in_cuda_memory = true;
  } else {
    cpu_triangles = main_engine_->GetMesh()->triangles;
  }

  ITMMesh::Triangle* triangleArray = cpu_triangles->GetData(MEMORYDEVICE_CPU);

  out_cloud->width = main_engine_->GetMesh()->noTotalTriangles *
                     3;  // Point cloud has at least 3 points per triangle.
  out_cloud->height = 1;
  out_cloud->is_dense = false;
  out_cloud->points.resize(out_cloud->width * out_cloud->height);

  ROS_INFO_STREAM("This mesh has " << main_engine_->GetMesh()->noTotalTriangles
                                   << " triangles");

  const std::string meshFileName = "test.stl";

  // All vertices of the mesh are stored in the pcl point cloud.
  for (int64 i = 0; i < main_engine_->GetMesh()->noTotalTriangles * 3;
       i = i + 3) {
    out_cloud->points[i].x = triangleArray[i].p0.x;
    out_cloud->points[i].y = triangleArray[i].p0.y;
    out_cloud->points[i].z = triangleArray[i].p0.z;

    out_cloud->points[i + 1].x = triangleArray[i].p1.x;
    out_cloud->points[i + 1].y = triangleArray[i].p1.y;
    out_cloud->points[i + 1].z = triangleArray[i].p1.z;

    out_cloud->points[i + 2].x = triangleArray[i].p2.x;
    out_cloud->points[i + 2].y = triangleArray[i].p2.y;
    out_cloud->points[i + 2].z = triangleArray[i].p2.z;
  }

  if (rm_triangle_in_cuda_memory) {
    delete cpu_triangles;
  }
}

bool RosEngine::publishMap(std_srvs::Empty::Request& request,
                           std_srvs::Empty::Response& response) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_pcl;
  sensor_msgs::PointCloud2 point_cloud_msg;

  extractMeshToPcl(point_cloud_pcl);
  pcl::toROSMsg(*point_cloud_pcl, point_cloud_msg);
  point_cloud_msg.header.frame_id = camera_frame_id_;
  point_cloud_msg.header.stamp = ros::Time::now();

  complete_point_cloud_pub_.publish(point_cloud_msg);

  return true;
}
}  // namespace Engine
}  // namespace InfiniTAM
#else

namespace InfiniTAM {
namespace Engine {

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
}  // namespace Engine
}  // namespace InfiniTAM

#endif
