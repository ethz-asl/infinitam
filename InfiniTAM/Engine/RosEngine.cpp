// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "RosEngine.h"

#include <Eigen/Dense>
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
  rgb_info_sub = nh.subscribe(rgb_camera_info_topic_, 1,
                              &RosEngine::rgbCameraInfoCallback,
                              static_cast<RosEngine*>(this));

  complete_point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(
      complete_cloud_topic_, 1);

  marker_pub_ = nh.advertise<visualization_msgs::Marker>(
      "/visualization_marker", 200);

  while (!rgb_info_ready_ || !depth_info_ready_) {
    ROS_INFO("Spinning, waiting for rgb and depth camera info messages.");
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  // initialize service
  publish_scene_service_ = nh.advertiseService("publish_scene",
                                               &RosEngine::publishMap,
                                               static_cast<RosEngine*>(this));

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

    cv_rgb_image_ = cv_bridge::toCvCopy(msg,
                                        sensor_msgs::image_encodings::RGB8);
  }
}

void RosEngine::depthCallback(const sensor_msgs::Image::ConstPtr& msg) {
  if (!depth_ready_ && data_available_) {
    std::lock_guard<std::mutex> guard(depth_mutex_);
    depth_ready_ = true;

    cv_depth_image_ = cv_bridge::toCvCopy(
        msg, sensor_msgs::image_encodings::TYPE_16UC1);
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
                             tf_world_to_camera_transform_current_);

    // get the first transformation between the world and the camera.
    if (first_time_tf_available_) {
      tf_world_to_camera_transform_at_start_ =
          tf_world_to_camera_transform_current_;
      // origin of the map for infinitam
      br.sendTransform(
          tf::StampedTransform(tf_world_to_camera_transform_at_start_,
                               ros::Time::now(), "world", "tf_camera_initial"));
      ROS_INFO("Got first tf Message");
      first_time_tf_available_ = false;
      okay_to_send = true;
    }

    // WORLD FRAME AND CURRENT CAMERA.
    tf::Matrix3x3 tf_world_to_current_camera_rotation;
    tf::Vector3 tf_world_to_current_camera_translation_in_world_frame,
        tf_world_to_current_camera_translation_in_infinitam_origin_frame;

    tf_world_to_current_camera_rotation = tf_world_to_camera_transform_current_
        .getBasis();
    tf_world_to_current_camera_translation_in_world_frame =
        tf_world_to_camera_transform_current_.getOrigin();

    // represent vector from world to infinitam origin into infinitam frame
    tf_world_to_current_camera_translation_in_infinitam_origin_frame =
        tf_world_to_current_camera_rotation
            * (tf_world_to_current_camera_translation_in_world_frame);

//    std::cout << "tf world to camera: \n"
//              << tf_world_to_current_camera_translation_in_world_frame.getX()
//              << " "
//              << tf_world_to_current_camera_translation_in_world_frame.getY()
//              << " "
//              << tf_world_to_current_camera_translation_in_world_frame.getZ()
//              << std::endl;
//    std::cout
//        << "tf world to camera: \n"
//        << tf_world_to_camera_transform_current_.getBasis().getColumn(0).getX()
//        << " "
//        << tf_world_to_camera_transform_current_.getBasis().getColumn(0).getY()
//        << " "
//        << tf_world_to_camera_transform_current_.getBasis().getColumn(0).getZ()
//        << "\n"
//        << tf_world_to_camera_transform_current_.getBasis().getColumn(1).getX()
//        << " "
//        << tf_world_to_camera_transform_current_.getBasis().getColumn(1).getY()
//        << " "
//        << tf_world_to_camera_transform_current_.getBasis().getColumn(1).getZ()
//        << "\n"
//        << tf_world_to_camera_transform_current_.getBasis().getColumn(2).getX()
//        << " "
//        << tf_world_to_camera_transform_current_.getBasis().getColumn(2).getY()
//        << " "
//        << tf_world_to_camera_transform_current_.getBasis().getColumn(2).getZ()
//        << std::endl;

    // WORLD FRAME AND INFINITAM ORIGIN.
    tf::Matrix3x3 tf_world_to_infinitam_origion_rotation;
    tf::Vector3 tf_world_to_infinitam_origin_translation_in_world_frame,
        tf_world_to_infinitam_origin_translation_in_infinitam_origin_frame;

    tf_world_to_infinitam_origion_rotation =
        tf_world_to_camera_transform_at_start_.getBasis();
    tf_world_to_infinitam_origin_translation_in_world_frame =
        tf_world_to_camera_transform_at_start_.getOrigin();

    // represent vector from world to infinitam origin into infinitam frame
    tf_world_to_infinitam_origin_translation_in_infinitam_origin_frame =
        tf_world_to_infinitam_origion_rotation
            * (tf_world_to_infinitam_origin_translation_in_world_frame);

//    ROS_INFO_STREAM(
//        "tf translation -rotated: \n" << -tf_relative_translation_rotated.getX() << " " << -tf_relative_translation_rotated.getY() << " " << -tf_relative_translation_rotated.getZ());
//    ROS_INFO_STREAM(
//        "tf rotation: \n" << tf_infinitam_origin_to_camera_transform_relative_.getBasis().getColumn(0).getX() << " " << tf_infinitam_origin_to_camera_transform_relative_.getBasis().getColumn(0).getY() << " " << tf_infinitam_origin_to_camera_transform_relative_.getBasis().getColumn(0).getZ() << "\n" << tf_infinitam_origin_to_camera_transform_relative_.getBasis().getColumn(1).getX() << " " << tf_infinitam_origin_to_camera_transform_relative_.getBasis().getColumn(1).getY() << " " << tf_infinitam_origin_to_camera_transform_relative_.getBasis().getColumn(1).getZ() << "\n" << tf_infinitam_origin_to_camera_transform_relative_.getBasis().getColumn(2).getX() << " " << tf_infinitam_origin_to_camera_transform_relative_.getBasis().getColumn(2).getY() << " " << tf_infinitam_origin_to_camera_transform_relative_.getBasis() .getColumn(2) .getZ());

    Vector3f infinitam_translation = main_engine_->GetTrackingState()->pose_d
        ->GetT();
    Matrix3f infinitam_rotation =
        main_engine_->GetTrackingState()->pose_d->GetR();
    tf::Matrix3x3 infinitam_rotation_;
    infinitam_rotation_.setValue(infinitam_rotation.m00, infinitam_rotation.m01,
                                 infinitam_rotation.m02, infinitam_rotation.m10,
                                 infinitam_rotation.m11, infinitam_rotation.m12,
                                 infinitam_rotation.m20, infinitam_rotation.m21,
                                 infinitam_rotation.m22);

    tf::Vector3 camera_translation_original, camera_translation_rotated;
    camera_translation_original.setX(infinitam_translation.x);
    camera_translation_original.setY(infinitam_translation.y);
    camera_translation_original.setZ(infinitam_translation.z);
    camera_translation_rotated = infinitam_rotation_
        * (camera_translation_original);

    infinitam_to_camera_transform_current_.frame_id_ = "tf_camera_initial";
    infinitam_to_camera_transform_current_.stamp_ = ros::Time::now();
    infinitam_to_camera_transform_current_.setOrigin(
        tf::Vector3(-camera_translation_rotated.getX(),
                    -camera_translation_rotated.getY(),
                    -camera_translation_rotated.getZ()));
    infinitam_to_camera_transform_current_.setBasis(infinitam_rotation_);
    std::cout
        << "infinitam_translation trafo-----------------------------------------: \n"
        << infinitam_to_camera_transform_current_.getOrigin().getX()
        << " "
        << infinitam_to_camera_transform_current_.getOrigin().getY()
        << " "
        << infinitam_to_camera_transform_current_.getOrigin().getZ()
        << std::endl;
    ROS_INFO_STREAM("infinitam_translation//////////////////////:\n" << infinitam_translation);
    ROS_INFO_STREAM("infinitam_rotation:\n" << infinitam_rotation);

    tf::Vector3 tf_world_camera_translation_test_in_world_frame,
        tf_infinitam_origin_to_current_camera_translation_test_in_world_frame,
        tf_infinitam_origin_to_current_camera_translation_test_in_infinitam_frame;

    tf_world_camera_translation_test_in_world_frame =
        tf_world_to_camera_transform_at_start_.getOrigin();

    tf_infinitam_origin_to_current_camera_translation_test_in_infinitam_frame =
        infinitam_to_camera_transform_current_.getOrigin();

    tf::Matrix3x3 tf_infinitam_origin_to_current_camera_rotation_test_in_infinitam_frame;
    tf_infinitam_origin_to_current_camera_rotation_test_in_infinitam_frame =
        infinitam_to_camera_transform_current_.getBasis();

    tf_infinitam_origin_to_current_camera_translation_test_in_world_frame =
        tf_world_to_camera_transform_at_start_.getBasis()
            * infinitam_to_camera_transform_current_.getOrigin();

    tf_world_camera_translation_test_in_world_frame =
        tf_world_camera_translation_test_in_world_frame
            + tf_infinitam_origin_to_current_camera_translation_test_in_world_frame;

    tf_world_to_camera_transform_current_test.setOrigin(
        tf_world_camera_translation_test_in_world_frame);
    tf_world_to_camera_transform_current_test.setBasis(
        tf_world_to_camera_transform_at_start_.getBasis()
            * infinitam_to_camera_transform_current_.getBasis());

//    std::cout << "tf world to camera test: \n"
//              << tf_world_to_camera_transform_current_test.getOrigin().getX()
//              << " "
//              << tf_world_to_camera_transform_current_test.getOrigin().getY()
//              << " "
//              << tf_world_to_camera_transform_current_test.getOrigin().getZ()
//              << std::endl;
//    std::cout
//        << "tf world to camera test: \n"
//        << tf_world_to_camera_transform_current_test.getBasis().getColumn(0)
//            .getX()
//        << " "
//        << tf_world_to_camera_transform_current_test.getBasis().getColumn(0)
//            .getY()
//        << " "
//        << tf_world_to_camera_transform_current_test.getBasis().getColumn(0)
//            .getZ()
//        << "\n"
//        << tf_world_to_camera_transform_current_test.getBasis().getColumn(1)
//            .getX()
//        << " "
//        << tf_world_to_camera_transform_current_test.getBasis().getColumn(1)
//            .getY()
//        << " "
//        << tf_world_to_camera_transform_current_test.getBasis().getColumn(1)
//            .getZ()
//        << "\n"
//        << tf_world_to_camera_transform_current_test.getBasis().getColumn(2)
//            .getX()
//        << " "
//        << tf_world_to_camera_transform_current_test.getBasis().getColumn(2)
//            .getY()
//        << " "
//        << tf_world_to_camera_transform_current_test.getBasis().getColumn(2)
//            .getZ()
//        << std::endl;

    // INFINITAM ORIGIN AND CURRENT CAMERA FRAME.
    tf::Matrix3x3 tf_infinitam_origin_to_camera_current_rotation;
    tf::Vector3
        tf_infinitam_origin_to_camera_current_translation_in_world_frame,
        tf_infinitam_origin_to_camera_current_translation_in_infinitam_frame,
        tf_infinitam_origin_to_camera_current_translation_in_infinitam_frame_rotated;
    tf_infinitam_origin_to_camera_transform_relative_.frame_id_ = "tf_camera_initial";
    tf_infinitam_origin_to_camera_transform_relative_.stamp_ = ros::Time::now();

    tf_infinitam_origin_to_camera_current_rotation =
        tf_world_to_camera_transform_at_start_.getBasis().inverse()
            * tf_world_to_camera_transform_current_.getBasis();

    tf_infinitam_origin_to_camera_current_translation_in_world_frame =
        -tf_world_to_camera_transform_at_start_.getOrigin()
            + tf_world_to_camera_transform_current_.getOrigin();

    tf_infinitam_origin_to_camera_current_translation_in_infinitam_frame =
        tf_world_to_camera_transform_at_start_.getBasis()
            * tf_infinitam_origin_to_camera_current_translation_in_world_frame;

    tf_infinitam_origin_to_camera_transform_relative_.setOrigin(
        tf_infinitam_origin_to_camera_current_translation_in_infinitam_frame);
    tf_infinitam_origin_to_camera_transform_relative_.setBasis(
        tf_infinitam_origin_to_camera_current_rotation);

    tf_infinitam_origin_to_camera_current_translation_in_infinitam_frame_rotated =
        tf_infinitam_origin_to_camera_current_rotation.inverse()
            * tf_infinitam_origin_to_camera_current_translation_in_infinitam_frame;
    std::cout
        << "tf trafo infinitam to camera test--------------------------------: \n"
        << tf_infinitam_origin_to_camera_current_translation_in_infinitam_frame.getX()
        << " "
        << tf_infinitam_origin_to_camera_current_translation_in_infinitam_frame.getY()
        << " "
        << tf_infinitam_origin_to_camera_current_translation_in_infinitam_frame.getZ()
        << std::endl;
    std::cout
        << "tf infinitam to camera test//////////////////////////////: \n"
        << -tf_infinitam_origin_to_camera_current_translation_in_infinitam_frame_rotated.getX()
        << " "
        << -tf_infinitam_origin_to_camera_current_translation_in_infinitam_frame_rotated.getY()
        << " "
        << -tf_infinitam_origin_to_camera_current_translation_in_infinitam_frame_rotated.getZ()
        << std::endl;
    std::cout
        << "tf infinitam to camera test: \n"
        << tf_infinitam_origin_to_camera_transform_relative_.getBasis()
            .getColumn(0).getX()
        << " "
        << tf_infinitam_origin_to_camera_transform_relative_.getBasis()
            .getColumn(0).getY()
        << " "
        << tf_infinitam_origin_to_camera_transform_relative_.getBasis()
            .getColumn(0).getZ()
        << "\n"
        << tf_infinitam_origin_to_camera_transform_relative_.getBasis()
            .getColumn(1).getX()
        << " "
        << tf_infinitam_origin_to_camera_transform_relative_.getBasis()
            .getColumn(1).getY()
        << " "
        << tf_infinitam_origin_to_camera_transform_relative_.getBasis()
            .getColumn(1).getZ()
        << "\n"
        << tf_infinitam_origin_to_camera_transform_relative_.getBasis()
            .getColumn(2).getX()
        << " "
        << tf_infinitam_origin_to_camera_transform_relative_.getBasis()
            .getColumn(2).getY()
        << " "
        << tf_infinitam_origin_to_camera_transform_relative_.getBasis()
            .getColumn(2).getZ()
        << std::endl;

    if (okay_to_send) {
      // origin of the map for infinitam
      br.sendTransform(
          tf::StampedTransform(tf_world_to_camera_transform_at_start_,
                               ros::Time::now(), "world", "tf_camera_initial"));
      // current position of the camera in tf
      br.sendTransform(
          tf::StampedTransform(tf_world_to_camera_transform_current_test,
                               ros::Time::now(), "world",
                               "infinitam_pose_test"));
      // current position of the camera from infinitam
      br.sendTransform(
          tf::StampedTransform(infinitam_to_camera_transform_current_,
                               ros::Time::now(), "tf_camera_initial",
                               "infinitam_pose"));
      br.sendTransform(
          tf::StampedTransform(
              tf_infinitam_origin_to_camera_transform_relative_,
              ros::Time::now(), "tf_camera_initial", "infinitam_pose_test2"));
    }

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
Vector2i RosEngine::getDepthImageSize(void) {
  return image_size_depth_;
}
Vector2i RosEngine::getRGBImageSize(void) {
  return image_size_rgb_;
}

void RosEngine::extractMeshToPcl(
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_pcl) {
  CHECK_NOTNULL(main_engine_);
  CHECK_NOTNULL(&point_cloud_pcl);
  set_camera_pose_ = false;

  main_engine_->GetMeshingEngine()->MeshScene(main_engine_->GetMesh(),
                                              main_engine_->GetScene());
  ROS_INFO("got mesh successfully");
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

  point_cloud_pcl->width = main_engine_->GetMesh()->noTotalTriangles * 3;  // Point cloud has at least 3 points per triangle.
  point_cloud_pcl->height = 1;
  point_cloud_pcl->is_dense = false;
  point_cloud_pcl->points.resize(
      point_cloud_pcl->width * point_cloud_pcl->height);

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
    bool RosEngine::hasMoreImages(void) {return false;}
    Vector2i RosEngine::getDepthImageSize(void) {return Vector2i(0, 0);}
    Vector2i RosEngine::getRGBImageSize(void) {return Vector2i(0, 0);}
  }  // namespace Engine
}  // namespace InfiniTAM

#endif
