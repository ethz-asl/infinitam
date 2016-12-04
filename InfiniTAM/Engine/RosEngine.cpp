// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "RosEngine.h"

#include <Eigen/Dense>
#include <cstdio>
#include <stdexcept>
#include <string>

#include <tf/transform_broadcaster.h>

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
      data_available_(true),
      tf_available_(true),
      debug_mode_(true),
      first_time_tf_available_(true),
      okay_to_send(false) {
  tf_pos_x = tf_pos_y = tf_pos_z = tf_rot_t = tf_rot_u = tf_rot_v = tf_rot_qx =
      tf_rot_qy = tf_rot_qz = tf_rot_qw = tf_rot_angle = 0;

  ros::Subscriber rgb_info_sub;
  ros::Subscriber depth_info_sub;
  set_camera_pose_ = true;
  nh.param<std::string>("rgb_camera_info_topic", rgb_camera_info_topic_,
                        "/camera/rgb/camera_info");
  nh.param<std::string>("depth_camera_info_topic", depth_camera_info_topic_,
                        "/camera/depth/camera_info");

  nh.param<std::string>("camera_frame_id", camera_frame_id_,
                        "sr300_depth_optical_frame");
  nh.param<std::string>("world_frame_id", world_frame_id_, "world");

  camera_pose_ = new ITMPose;

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

  marker_pub_ =
      nh.advertise<visualization_msgs::Marker>("/visualization_marker", 200);

  while (!rgb_info_ready_ || !depth_info_ready_) {
    ROS_INFO("Spinning, waiting for rgb and depth camera info messages.");
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

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
  //  delete camera_pose_;
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
void RosEngine::TFCallback(const tf::tfMessage& tf_msg) {
  try {
    // get transform from world to camera frame.
    listener.lookupTransform(world_frame_id_, camera_frame_id_, ros::Time(0),
                             camera_world_transform_current_);

    // get the first transformation between the world and the camera.
    if (first_time_tf_available_) {
      camera_world_transform_at_start_ = camera_world_transform_current_;
      ROS_INFO("Got first tf Message");
      first_time_tf_available_ = false;
      okay_to_send = true;
    }

    // calculate the relative transformation between start and current position.
    camera_world_transform_relative_.setOrigin(
        camera_world_transform_current_.getOrigin() -
        camera_world_transform_at_start_.getOrigin());

    camera_world_transform_relative_.setRotation(
        camera_world_transform_current_.getRotation() -
        camera_world_transform_at_start_.getRotation());

    // rotate vector, bring vector from world frame to initial sr300 frame.
    Eigen::Quaterniond q(camera_world_transform_at_start_.getRotation().getX(),
                         camera_world_transform_at_start_.getRotation().getY(),
                         camera_world_transform_at_start_.getRotation().getZ(),
                         camera_world_transform_at_start_.getRotation().getW());
    q.normalize();
    Eigen::Vector3d v;
    v << camera_world_transform_relative_.getOrigin().getX(),
        camera_world_transform_relative_.getOrigin().getY(),
        camera_world_transform_relative_.getOrigin().getZ();
    Eigen::Quaterniond p;
    p.w() = 0;
    p.vec() = v;
    Eigen::Quaterniond rotatedP = q * p * q.inverse();
    Eigen::Vector3d rotatedV = rotatedP.vec();

    // calculate the relative pose of the camera.
    tf_pos_x = camera_world_transform_current_.getOrigin().getX();
    tf_pos_y = camera_world_transform_current_.getOrigin().getY();
    tf_pos_z = camera_world_transform_current_.getOrigin().getZ();
    tf_rot_qx = camera_world_transform_current_.getRotation().getX();
    tf_rot_qy = camera_world_transform_current_.getRotation().getY();
    tf_rot_qz = camera_world_transform_current_.getRotation().getZ();
    tf_rot_qw = camera_world_transform_current_.getRotation().getW();
    tf_rot_axis = camera_world_transform_current_.getRotation().getAxis();
    tf_rot_angle = camera_world_transform_current_.getRotation().getAngle();
    tf_rot_t = tf_rot_axis.getX();
    tf_rot_u = tf_rot_axis.getY();
    tf_rot_v = tf_rot_axis.getZ();

    ROS_INFO_STREAM("tf:       "
                    << " tx:" << tf_pos_x << " ty:" << tf_pos_y
                    << " tz:" << tf_pos_z << " rx:" << tf_rot_t
                    << " ry:" << tf_rot_u << " rz:" << tf_rot_v);

    main_engine_->GetTrackingState()->pose_d->GetParams(tra, rot);
    infinitam_pos_x = tra.x;
    infinitam_pos_y = tra.y;
    infinitam_pos_z = tra.z;

    infinitam_rot_x = rot.x;
    infinitam_rot_y = rot.y;
    infinitam_rot_z = rot.z;
    ROS_INFO_STREAM("infinitam:"
                    << " tx:" << infinitam_pos_x << " ty:" << infinitam_pos_y
                    << " tz:" << infinitam_pos_z << " rx:" << infinitam_rot_x
                    << " ry:" << infinitam_rot_y << " rz:" << infinitam_rot_z);
    double angle;
    angle = sqrt(infinitam_rot_x * infinitam_rot_x +
                 infinitam_rot_y * infinitam_rot_y +
                 infinitam_rot_z * infinitam_rot_z);
    tf::Vector3 vec;
    vec.setX(infinitam_rot_x);
    vec.setX(infinitam_rot_y);
    vec.setX(infinitam_rot_z);
    vec.normalize();

    tf_initial_current_camera_transform_.frame_id_ = "tf_sr300_initial";
    tf_initial_current_camera_transform_.stamp_ = ros::Time::now();
    tf_initial_current_camera_transform_.setOrigin(
        tf::Vector3(rotatedV[0], rotatedV[1], rotatedV[2]));
    tf::Quaternion pose_d_quad2;
    pose_d_quad2.setRotation(vec, angle);
    tf_initial_current_camera_transform_.setRotation(pose_d_quad2);

    initial_current_camera_transform_.frame_id_ = "tf_sr300_initial";
    initial_current_camera_transform_.stamp_ = ros::Time::now();
    initial_current_camera_transform_.setOrigin(
        tf::Vector3(infinitam_pos_x, infinitam_pos_y, infinitam_pos_z));
    tf::Quaternion pose_d_quad;
    pose_d_quad.setRotation(vec, angle);
    initial_current_camera_transform_.setRotation(pose_d_quad);

    if (okay_to_send) {
      tf::TransformBroadcaster br;
      // origin of the map for infinitam
      br.sendTransform(tf::StampedTransform(camera_world_transform_at_start_,
                                            ros::Time::now(), "world",
                                            "tf_sr300_initial"));
      // current position of the camera
      br.sendTransform(tf::StampedTransform(
          tf_initial_current_camera_transform_, ros::Time::now(),
          "tf_sr300_initial", "tf_sr300_relative"));
      br.sendTransform(tf::StampedTransform(
          initial_current_camera_transform_, ros::Time::now(),
          "tf_sr300_initial", "infinitam_pose"));
      ROS_INFO("tf published");
    }

    //    camera_pose_->SetFrom(tf_pos_x, tf_pos_y, tf_pos_z, tf_rot_t,
    //    tf_rot_u,
    //                          tf_rot_v);
    camera_pose_->SetFrom(tf_pos_x, tf_pos_y, tf_pos_z, 0, 0, 0);

    if (set_camera_pose_) {
      // TODO(gocarlos): stop writing pose when camera has done its work.
      // currently pose is set also after rosbag has finished.
      //            main_engine_->GetTrackingState()->pose_d = camera_pose_;
      //            main_engine_->GetTrackingState()->pose_pointCloud =
      //            camera_pose_;
    }
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
  }
}

void RosEngine::getMeasurement(ITMPose* pose) {
  ROS_INFO("getMeasurement().");
  if (!tf_available_) {
    return;
  }
  tf_available_ = false;

  pose = camera_pose_;

  //  std::lock_guard < std::mutex > tf_guard(tf_mutex_);
  //  tf_ready_ = false;

  tf_available_ = true;
}

bool RosEngine::hasMoreMeasurements(void) {
  ROS_INFO("hasMoreMeasurements().");

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_pcl) {
  CHECK_NOTNULL(main_engine_);
  CHECK_NOTNULL(&point_cloud_pcl);
  set_camera_pose_ = false;

  main_engine_->GetMeshingEngine()->MeshScene(main_engine_->GetMesh(),
                                              main_engine_->GetScene());
  ROS_INFO("got mesh succefully");
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
  ROS_INFO("got cpu_triangles");

  ITMMesh::Triangle* triangleArray = cpu_triangles->GetData(MEMORYDEVICE_CPU);
  ROS_INFO("got triangleArray");

  point_cloud_pcl->width =
      main_engine_->GetMesh()->noTotalTriangles *
      3;  // Point cloud has at least 3 points per triangle.
  point_cloud_pcl->height = 1;
  point_cloud_pcl->is_dense = false;
  point_cloud_pcl->points.resize(point_cloud_pcl->width *
                                 point_cloud_pcl->height);

  ROS_INFO_STREAM("This mesh has " << point_cloud_pcl->width << " triangles");

  // All vertices of the mesh are stored in the pcl point cloud.
  for (int64 i = 0; i < main_engine_->GetMesh()->noTotalTriangles * 3;
       i = i + 3) {
    point_cloud_pcl->points[i].x = triangleArray[i].p0.x;
    point_cloud_pcl->points[i].y = triangleArray[i].p0.y;
    point_cloud_pcl->points[i].z = triangleArray[i].p0.z;

    point_cloud_pcl->points[i + 1].x = triangleArray[i].p1.x;
    point_cloud_pcl->points[i + 1].y = triangleArray[i].p1.y;
    point_cloud_pcl->points[i + 1].z = triangleArray[i].p1.z;

    point_cloud_pcl->points[i + 2].x = triangleArray[i].p2.x;
    point_cloud_pcl->points[i + 2].y = triangleArray[i].p2.y;
    point_cloud_pcl->points[i + 2].z = triangleArray[i].p2.z;
  }
  ROS_INFO("got out_cloud");

  if (rm_triangle_in_cuda_memory) {
    delete cpu_triangles;
  }
}

bool RosEngine::publishMap(std_srvs::Empty::Request& request,
                           std_srvs::Empty::Response& response) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_pcl(
      new pcl::PointCloud<pcl::PointXYZ>);
  ROS_INFO("publishMap");

  extractMeshToPcl(point_cloud_pcl);

  ROS_INFO("got point cloud");

  sensor_msgs::PointCloud2 point_cloud_msg;
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
