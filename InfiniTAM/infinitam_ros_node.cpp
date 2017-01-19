// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include <glog/logging.h>

#include <cstdlib>
#include <string>

#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PolygonMesh.h>

#include <geometry_msgs/TransformStamped.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <shape_msgs/Mesh.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

#include "Engine/CLIEngine.h"
#include "Engine/ImageSourceEngine.h"
#include "Engine/Kinect2Engine.h"
#include "Engine/LibUVCEngine.h"
#include "Engine/OpenNIEngine.h"
#include "Engine/PoseSourceEngine.h"
#include "Engine/RealSenseEngine.h"
#include "Engine/RosImageSourceEngine.h"
#include "Engine/RosPoseSourceEngine.h"
#include "Engine/UIEngine.h"

using namespace InfiniTAM::Engine;

/** Create a default source of depth images from a list of command line
 arguments. Typically, @para arg1 would identify the calibration file to
 use, @para arg2 the colour images, @para arg3 the depth images and
 @para arg4 the IMU images. If images are omitted, some live sources will
 be tried.
 */
class InfinitamNode {
 public:
  InfinitamNode(int& argc, char** argv);
  ~InfinitamNode();

  //! Read parameters from the ROS parameter server.
  void readParameters();

  //! Choose Image and Pose sources.
  void SetUpSources();

  //! ROS Service Callback method, initializes Infinitam
  bool startInfinitam(std_srvs::SetBool::Request& request,
                      std_srvs::SetBool::Response& response);

  //! ROS Service Callback method, make infinitam publish the current map.
  bool publishMap(std_srvs::Empty::Request& request,
                  std_srvs::Empty::Response& response);

  //! Converts the internal Mesh to a PCL point cloud.
  void extractITMMeshToPclCloud(
      const ITMMesh::Triangle& triangleArray,
      pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_pcl);

  //! Converts the internal Mesh to a PCL PolygonMesh.
  void extractITMMeshToPolygonMesh(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_pcl,
      pcl::PolygonMesh::Ptr polygon_mesh_ptr);

  //! Converts a PCL PolygonMesh to a ROS Mesh.
  bool convertPolygonMeshToRosMesh(const pcl::PolygonMesh::Ptr in,
                                   shape_msgs::Mesh::Ptr mesh);

 private:
  ros::NodeHandle node_handle_;

  const char* arg1 = "";
  const char* arg2 = nullptr;
  const char* arg3 = nullptr;
  const char* arg4 = nullptr;
  int argc;
  char** argv;

  ITMMainEngine* main_engine_ = nullptr;
  ITMLibSettings* internal_settings_ = nullptr;
  ImageSourceEngine* image_source_ = nullptr;
  IMUSourceEngine* imu_source_ = nullptr;
  PoseSourceEngine* pose_source_ = nullptr;

  ros::Subscriber rgb_sub_;
  ros::Subscriber depth_sub_;
  ros::Subscriber tf_sub_;
  std::string rgb_image_topic;
  std::string depth_image_topic;

  //! Name for the depth camera frame id in TF.
  std::string camera_frame_id_;
  //! Name for the world fixed frame id in TF.
  std::string world_frame_id_;

  //! Initialize Infinitam
  ros::ServiceServer start_infinitam_service_;
  //! Publish the mesh build in infinitam when called.
  ros::ServiceServer publish_mesh_service_;

  //! ROS publisher to send out the complete cloud.
  ros::Publisher complete_point_cloud_pub_;
  //! ROS topic name where the generated complete cloud is published.
  std::string complete_cloud_topic_;

  //! ROS publisher to send out the complete cloud as ROS mesh.
  ros::Publisher complete_mesh_pub_;
  //! ROS topic name where the generated complete mesh is published.
  std::string complete_mesh_topic_;

  //! ROS Mesh of the map.
  shape_msgs::Mesh::Ptr ros_scene_mesh_ptr_;

  //! Set to true if one wants to save the mesh to the file system.
  bool save_cloud_to_file_system_;

  //! PCL Mesh of the map
  pcl::PolygonMesh::Ptr mesh_ptr_;
};

InfinitamNode::InfinitamNode(int& argc, char** argv) : node_handle_("~") {
  this->argc = argc;
  this->argv = argv;

  pose_source_ = new PoseSourceEngine();
  internal_settings_ = new ITMLibSettings();
  mesh_ptr_.reset(new pcl::PolygonMesh);
  ros_scene_mesh_ptr_.reset(new shape_msgs::Mesh);

  readParameters();

  // Initialize service.
  start_infinitam_service_ = node_handle_.advertiseService(
      "start_infinitam", &InfinitamNode::startInfinitam, this);

  publish_mesh_service_ = node_handle_.advertiseService(
      "publish_mesh", &InfinitamNode::publishMap, this);

  // ROS publishers.
  complete_point_cloud_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(
      complete_cloud_topic_, 5);

  complete_mesh_pub_ =
      node_handle_.advertise<shape_msgs::Mesh>(complete_mesh_topic_, 5);
}

InfinitamNode::~InfinitamNode() {
  delete main_engine_;
  delete internal_settings_;
  delete image_source_;
  delete pose_source_;
  if (imu_source_ != nullptr) delete imu_source_;
}

bool InfinitamNode::startInfinitam(std_srvs::SetBool::Request& request,
                                   std_srvs::SetBool::Response& response) {
  LOG(INFO) << "startInfinitam start!";

  // Turn on infinitam
  if (request.data) {
    int arg = 1;
    do {
      if (argv[arg] != nullptr)
        arg1 = argv[arg];
      else
        break;
      ++arg;
      if (argv[arg] != nullptr)
        arg2 = argv[arg];
      else
        break;
      ++arg;
      if (argv[arg] != nullptr)
        arg3 = argv[arg];
      else
        break;
      ++arg;
      if (argv[arg] != nullptr)
        arg4 = argv[arg];
      else
        break;
    } while (false);
    printf("after while\n");

    if (arg == 1) {
      printf(
          "usage: %s [<calibfile> [<imagesource>] ]\n"
          "  <calibfile>   : path to a file containing intrinsic calibration "
          "parameters\n"
          "  <imagesource> : either one argument to specify OpenNI device ID\n"
          "                  or two arguments specifying rgb and depth file "
          "masks\n"
          "\n"
          "examples:\n"
          "  %s ./Files/Teddy/calib.txt ./Files/Teddy/Frames/%%04i.ppm "
          "./Files/Teddy/Frames/%%04i.pgm\n"
          "  %s ./Files/Teddy/calib.txt\n\n",
          argv[0], argv[0], argv[0]);
    }

    ROS_INFO_STREAM("initialising ...");

    SetUpSources();

    main_engine_ = new ITMMainEngine(internal_settings_, &image_source_->calib,
                                     image_source_->getRGBImageSize(),
                                     image_source_->getDepthImageSize());

    if (image_source_ == nullptr) {
      std::cout << "failed to open any image stream" << std::endl;
    }

    image_source_->main_engine_ = main_engine_;
    pose_source_->main_engine_ = main_engine_;

    // TODO(gocarlos): it would be nice to output the name of the enum here
    // instead of intigers.
    std::cout << "Using device: " << internal_settings_->deviceType
              << std::endl;
    std::cout << "Using tracker: " << internal_settings_->trackerType
              << std::endl;

    UIEngine::Instance()->Initialise(argc, argv, image_source_, imu_source_,
                                     main_engine_, "./Files/Out",
                                     internal_settings_->deviceType);

    // Start already with processing once the run method is called.
    UIEngine::Instance()->mainLoopAction = UIEngine::PROCESS_VIDEO;
    ROS_INFO("GUI Engine Initialized.");
    UIEngine::Instance()->Run();
    ROS_INFO("Done.");
    static_cast<RosPoseSourceEngine*>(pose_source_)->set_camera_pose_ = false;
    UIEngine::Instance()->Shutdown();
  }

  // Turn off infinitam.
  if (!request.data) {
    UIEngine::Instance()->mainLoopAction = UIEngine::PROCESS_PAUSED;
    UIEngine::Instance()->mainLoopAction = UIEngine::EXIT;
  }

  return true;
}

bool InfinitamNode::publishMap(std_srvs::Empty::Request& request,
                               std_srvs::Empty::Response& response) {
  ROS_INFO_STREAM("Service for publishing the map has started.");
  // Make the mesh ready for reading.
  main_engine_->GetMeshingEngine()->MeshScene(main_engine_->GetMesh(),
                                              main_engine_->GetScene());

  // Get triangles from the device's memory.
  ORUtils::MemoryBlock<ITMMesh::Triangle>* cpu_triangles;
  bool rm_triangle_from_cuda_memory = false;
  if (main_engine_->GetMesh()->memoryType == MEMORYDEVICE_CUDA) {
    cpu_triangles = new ORUtils::MemoryBlock<ITMMesh::Triangle>(
        main_engine_->GetMesh()->noMaxTriangles, MEMORYDEVICE_CPU);
    cpu_triangles->SetFrom(
        main_engine_->GetMesh()->triangles,
        ORUtils::MemoryBlock<ITMMesh::Triangle>::CUDA_TO_CPU);
    rm_triangle_from_cuda_memory = true;
  } else {
    cpu_triangles = main_engine_->GetMesh()->triangles;
  }

  // Read the memory and store it in a new array.
  ITMMesh::Triangle* triangle_array = cpu_triangles->GetData(MEMORYDEVICE_CPU);

  ROS_ERROR_COND(main_engine_->GetMesh()->noTotalTriangles < 1,
                 "The mesh has too few triangles, only: %d",
                 main_engine_->GetMesh()->noTotalTriangles);

  // Publish point cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_pcl(
      new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::PointCloud2 point_cloud_msg;

  extractITMMeshToPclCloud(*triangle_array, point_cloud_pcl);
  ROS_INFO_STREAM("Got Point Cloud");

  pcl::toROSMsg(*point_cloud_pcl, point_cloud_msg);
  if (static_cast<RosPoseSourceEngine*>(pose_source_)->got_tf_msg_) {
    // If we are using the External tracker then the cloud is in the
    // world frame.
    point_cloud_msg.header.frame_id = world_frame_id_;
  } else {
    // If we are not using the External Tracker put the cloud in the normal
    // camera frame id.
    point_cloud_msg.header.frame_id = camera_frame_id_;
  }

  point_cloud_msg.header.stamp = ros::Time::now();
  complete_point_cloud_pub_.publish(point_cloud_msg);

  ROS_INFO_STREAM("Going to extract PolygonMesh");
  // Get the Mesh as PCL PolygonMesh .
  extractITMMeshToPolygonMesh(point_cloud_pcl, mesh_ptr_);

  ROS_INFO_STREAM("Loaded a PolygonMesh with "
                  << mesh_ptr_->cloud.width * mesh_ptr_->cloud.height
                  << " points and " << mesh_ptr_->polygons.size()
                  << " polygons.");

  // Convert PCL PolygonMesh into ROS shape_msgs Mesh.
  convertPolygonMeshToRosMesh(mesh_ptr_, ros_scene_mesh_ptr_);
  ROS_INFO_STREAM("Got ROS Mesh.");

  // Publish ROS Mesh.
  complete_mesh_pub_.publish(*ros_scene_mesh_ptr_);
  ROS_INFO_STREAM("ROS Mesh published.");

  if (save_cloud_to_file_system_) {
    const std::string filename_stl_file =
        ros::package::getPath("infinitam") + "/scenes/scene_mesh" + ".stl";
    main_engine_->GetMesh()->WriteSTL(filename_stl_file.c_str());
  }

  if (rm_triangle_from_cuda_memory) {
    delete cpu_triangles;
  }
  ROS_INFO_STREAM("Service for publishing the map has ended!");
  return true;
}

void InfinitamNode::extractITMMeshToPolygonMesh(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_pcl,
    pcl::PolygonMesh::Ptr polygon_mesh_ptr) {
  std::size_t nr_triangles = 0u;
  std::size_t nr_points = 0u;
  nr_triangles = main_engine_->GetMesh()->noTotalTriangles;
  nr_points = nr_triangles * 3u;
  ROS_INFO_STREAM("nr_triangles:  " << nr_triangles);
  ROS_INFO_STREAM("nr_points:  " << nr_points);

  // Build the point cloud.
  pcl::toPCLPointCloud2(*point_cloud_pcl, polygon_mesh_ptr->cloud);

  // write vertices
  ROS_DEBUG_STREAM("Going to fill the mesh with faces.");

  mesh_ptr_->polygons.resize(nr_triangles);

  for (std::size_t i = 0u; i < nr_triangles; ++i) {
    //  Write faces.
    mesh_ptr_->polygons[i].vertices.resize(3u);
    polygon_mesh_ptr->polygons[i].vertices[0] = (i * 3 + 2);
    polygon_mesh_ptr->polygons[i].vertices[1] = (i * 3 + 1);
    polygon_mesh_ptr->polygons[i].vertices[2] = (i * 3 + 0);
  }

  ROS_INFO_STREAM("cloud filled: header: "
                  << mesh_ptr_->cloud.header
                  << "height: " << polygon_mesh_ptr->cloud.height
                  << " width: " << polygon_mesh_ptr->cloud.width
                  << " fields.size: " << polygon_mesh_ptr->cloud.fields.size());

  ROS_INFO_STREAM("Polygons vector size: " << mesh_ptr_->polygons.size());
}

void InfinitamNode::extractITMMeshToPclCloud(
    const ITMMesh::Triangle& triangle_array,
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_pcl) {
  ROS_INFO("PCL PointCloud extraction from ITMMesh started.");
  CHECK_NOTNULL(main_engine_);
  CHECK_NOTNULL(&point_cloud_pcl);

  std::size_t nr_triangles = 0u;
  std::size_t nr_points = 0u;
  nr_triangles = main_engine_->GetMesh()->noTotalTriangles;
  // Infinitam stores the mesh as a set of triangles,
  // number of points is 3x number of triangles. See ITMMesh.h
  nr_points = nr_triangles * 3u;

  // Point cloud has at least 3 points per triangle.
  point_cloud_pcl->width = nr_points;
  point_cloud_pcl->height = 1u;
  point_cloud_pcl->is_dense = true;
  point_cloud_pcl->points.resize(point_cloud_pcl->width *
                                 point_cloud_pcl->height);

  ROS_ERROR_COND(main_engine_->GetMesh()->noTotalTriangles < 1u,
                 "The mesh has too few triangles, only: %d",
                 main_engine_->GetMesh()->noTotalTriangles);

  std::size_t point_number = 0u;

  // All vertices of the mesh are stored in the PCL point cloud.
  for (std::size_t i = 0u; i < nr_triangles; ++i) {
    point_cloud_pcl->points[point_number].x = (&triangle_array)[i].p0.x;
    point_cloud_pcl->points[point_number].y = (&triangle_array)[i].p0.y;
    point_cloud_pcl->points[point_number].z = (&triangle_array)[i].p0.z;
    ++point_number;
    point_cloud_pcl->points[point_number].x = (&triangle_array)[i].p1.x;
    point_cloud_pcl->points[point_number].y = (&triangle_array)[i].p1.y;
    point_cloud_pcl->points[point_number].z = (&triangle_array)[i].p1.z;
    ++point_number;
    point_cloud_pcl->points[point_number].x = (&triangle_array)[i].p2.x;
    point_cloud_pcl->points[point_number].y = (&triangle_array)[i].p2.y;
    point_cloud_pcl->points[point_number].z = (&triangle_array)[i].p2.z;
    ++point_number;
  }
  ROS_INFO("PCL PointCloud extraction from ITMMesh ended.");
}

bool InfinitamNode::convertPolygonMeshToRosMesh(
    const pcl::PolygonMesh::Ptr polygon_mesh_ptr,
    shape_msgs::Mesh::Ptr ros_mesh_ptr) {
  ROS_INFO("Conversion from PCL PolygonMesh to ROS Mesh started.");

  pcl_msgs::PolygonMesh pcl_msg_mesh;

  pcl_conversions::fromPCL(*polygon_mesh_ptr, pcl_msg_mesh);

  sensor_msgs::PointCloud2Modifier pcd_modifier(pcl_msg_mesh.cloud);

  size_t size = pcd_modifier.size();

  ros_mesh_ptr->vertices.resize(size);

  ROS_INFO_STREAM("polys: " << pcl_msg_mesh.polygons.size()
                            << " vertices: " << pcd_modifier.size());

  sensor_msgs::PointCloud2ConstIterator<float> pt_iter(pcl_msg_mesh.cloud, "x");

  for (size_t i = 0u; i < size; i++, ++pt_iter) {
    ros_mesh_ptr->vertices[i].x = pt_iter[0];
    ros_mesh_ptr->vertices[i].y = pt_iter[1];
    ros_mesh_ptr->vertices[i].z = pt_iter[2];
  }

  ROS_INFO_STREAM("Updated vertices");

  ros_mesh_ptr->triangles.resize(polygon_mesh_ptr->polygons.size());

  for (size_t i = 0u; i < polygon_mesh_ptr->polygons.size(); ++i) {
    if (polygon_mesh_ptr->polygons[i].vertices.size() < 3u) {
      ROS_WARN_STREAM("Not enough points in polygon. Ignoring it.");
      continue;
    }

    for (size_t j = 0u; j < 3u; ++j) {
      ros_mesh_ptr->triangles[i].vertex_indices[j] =
          polygon_mesh_ptr->polygons[i].vertices[j];
    }
  }
  ROS_INFO("Conversion from PCL PolygonMesh to ROS Mesh ended.");
  return true;
}

void InfinitamNode::readParameters() {
  // Set ROS topic names.
  node_handle_.param<std::string>("rgb_image_topic", rgb_image_topic,
                                  "/camera/rgb/image_raw");
  node_handle_.param<std::string>("depth_image_topic", depth_image_topic,
                                  "/camera/depth/image_raw");
  node_handle_.param<std::string>("scene_point_cloud", complete_cloud_topic_,
                                  "/scene_point_cloud");
  node_handle_.param<std::string>("scene_mesh", complete_mesh_topic_,
                                  "/scene_mesh");

  // Set the output one wants from Infinitam.
  node_handle_.param<bool>("save_cloud_to_file_system",
                           save_cloud_to_file_system_, true);

  // Set InfiniTAM settings.
  node_handle_.param<float>("viewFrustum_min",
                            internal_settings_->sceneParams.viewFrustum_min,
                            0.35f);
  node_handle_.param<float>(
      "viewFrustum_max", internal_settings_->sceneParams.viewFrustum_max, 3.0f);

  int tracker;
  node_handle_.param<int>("trackerType", tracker, 1);
  internal_settings_->trackerType =
      static_cast<ITMLibSettings::TrackerType>(tracker);

  node_handle_.param<std::string>("camera_frame_id", camera_frame_id_,
                                  "sr300_depth_optical_frame");
  node_handle_.param<std::string>("world_frame_id", world_frame_id_, "world");
}

void InfinitamNode::SetUpSources() {
  CHECK_NOTNULL(pose_source_);
  CHECK_NOTNULL(internal_settings_);

  const char* calibration_filename = arg1;
  const char* depth_image_filename = arg2;
  const char* rgb_image_filename = arg3;
  const char* filename_imu = arg4;

  printf("using calibration file: %s\n", calibration_filename);

  if (rgb_image_filename != nullptr) {
    printf("using rgb images: %s\nusing depth images: %s\n",
           depth_image_filename, rgb_image_filename);
    if (filename_imu == nullptr) {
      image_source_ = new ImageFileReader(
          calibration_filename, depth_image_filename, rgb_image_filename);
    } else {
      printf("using imu data: %s\n", filename_imu);
      image_source_ =
          new RawFileReader(calibration_filename, depth_image_filename,
                            rgb_image_filename, Vector2i(320, 240), 0.5f);
      imu_source_ = new IMUSourceEngine(filename_imu);
    }
  }

  if (image_source_ == nullptr) {
    printf("trying OpenNI device: %s\n", (depth_image_filename == nullptr)
                                             ? "<OpenNI default device>"
                                             : depth_image_filename);
    image_source_ =
        new OpenNIEngine(calibration_filename, depth_image_filename);
    if (image_source_->getDepthImageSize().x == 0) {
      delete image_source_;
      image_source_ = nullptr;
    }
  }
  if (image_source_ == nullptr) {
    printf("trying UVC device\n");
    image_source_ = new LibUVCEngine(calibration_filename);
    if (image_source_->getDepthImageSize().x == 0) {
      delete image_source_;
      image_source_ = nullptr;
    }
  }

  if (image_source_ == nullptr) {
    printf("trying MS Kinect 2 device\n");
    image_source_ = new Kinect2Engine(calibration_filename);
    if (image_source_->getDepthImageSize().x == 0) {
      delete image_source_;
      image_source_ = nullptr;
    }
  }
  if (image_source_ == nullptr) {
    printf("Checking if there are suitable ROS messages being published.\n");

    image_source_ =
        new RosImageSourceEngine(node_handle_, calibration_filename);

    // Get images from ROS topic.
    rgb_sub_ = node_handle_.subscribe(rgb_image_topic, 10,
                                      &RosImageSourceEngine::rgbCallback,
                                      (RosImageSourceEngine*)image_source_);

    depth_sub_ = node_handle_.subscribe(depth_image_topic, 10,
                                        &RosImageSourceEngine::depthCallback,
                                        (RosImageSourceEngine*)image_source_);

    // If the tracker is set to External, set the camera position from the ros
    // tf msg. If not set to External, the tf are received and used to transform
    // output mesh but not used to transform the camera during mapping.
    pose_source_ = new RosPoseSourceEngine(node_handle_);
    if (internal_settings_->trackerType == ITMLibSettings::TRACKER_EXTERNAL) {
      static_cast<RosPoseSourceEngine*>(pose_source_)->set_camera_pose_ = true;
    }

    tf_sub_ =
        node_handle_.subscribe("/tf", 10, &RosPoseSourceEngine::TFCallback,
                               (RosPoseSourceEngine*)pose_source_);

    if (image_source_->getDepthImageSize().x == 0) {
      delete image_source_;
      image_source_ = nullptr;
    }
  }

  // this is a hack to ensure backwards compatibility in certain configurations
  if (image_source_ == nullptr) {
    return;
  }
  if (image_source_->calib.disparityCalib.params == Vector2f(0.0f, 0.0f)) {
    image_source_->calib.disparityCalib.type = ITMDisparityCalib::TRAFO_AFFINE;
    image_source_->calib.disparityCalib.params = Vector2f(1.0f / 1000.0f, 0.0f);
  }
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  ros::init(argc, argv, "infinitamNode");
  InfinitamNode infinitamNode(argc, argv);

  while (ros::ok()) {
    ros::spin();
  }

  return EXIT_SUCCESS;
}
