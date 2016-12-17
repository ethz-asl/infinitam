// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include <glog/logging.h>
#include <math.h>
#include <cstdlib>
#include <string>

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

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <shape_msgs/Mesh.h>
//#include <shapes.h>
#include <geometric_shapes/shapes.h>

#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

// TEST
#include <visualization_msgs/Marker.h>

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

  //! ROS Service Callback method, initialises Infinitam
  bool startInfinitam(std_srvs::SetBool::Request& request,
                      std_srvs::SetBool::Response& response);

  //! ROS Service Callback method, make infinitam publish the current map.
  bool publishMap(std_srvs::Empty::Request& request,
                  std_srvs::Empty::Response& response);

  void extractMeshToPcl(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_pcl);

  void extractMeshToRosMesh(shape_msgs::Mesh::Ptr ros_mesh);

 private:
  ros::NodeHandle node_handle_;

  const char* arg1 = "";
  const char* arg2 = NULL;
  const char* arg3 = NULL;
  const char* arg4 = NULL;
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

  // initialize service
  ros::ServiceServer start_infinitam_service_;

  ros::ServiceServer build_mesh_service_;

  ros::ServiceServer publish_mesh_service_;

  //! ROS publisher to send out the complete cloud as ROS point cloud.
  ros::Publisher complete_point_cloud_pub_;
  //! ROS topic name where the generated complete cloud is published.
  std::string complete_cloud_topic_;

  //! ROS publisher to send out the complete cloud as ROS mesh.
  ros::Publisher complete_mesh_pub_;
  //! ROS topic name where the generated complete mesh is published.
  std::string complete_mesh_topic_;
};

InfinitamNode::InfinitamNode(int& argc, char** argv)
    : node_handle_("~") {
  this->argc = argc;
  this->argv = argv;

  pose_source_ = new PoseSourceEngine();
  internal_settings_ = new ITMLibSettings();

  readParameters();

  // initialize service
  start_infinitam_service_ = node_handle_.advertiseService(
      "start_infinitam", &InfinitamNode::startInfinitam, this);

  publish_mesh_service_ = node_handle_.advertiseService(
      "publish_mesh", &InfinitamNode::publishMap, this);

  complete_point_cloud_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(
      complete_cloud_topic_, 1);

  complete_mesh_pub_ = node_handle_.advertise<shape_msgs::Mesh>(
      complete_mesh_topic_, 1);
}

InfinitamNode::~InfinitamNode() {
  delete main_engine_;
  delete internal_settings_;
  delete image_source_;
  if (imu_source_ != NULL)
    delete imu_source_;
}
bool InfinitamNode::startInfinitam(std_srvs::SetBool::Request& request,
                                   std_srvs::SetBool::Response& response) {
  // turn on infinitam
  if (request.data) {
    int arg = 1;
    do {
      if (argv[arg] != NULL)
        arg1 = argv[arg];
      else
        break;
      ++arg;
      if (argv[arg] != NULL)
        arg2 = argv[arg];
      else
        break;
      ++arg;
      if (argv[arg] != NULL)
        arg3 = argv[arg];
      else
        break;
      ++arg;
      if (argv[arg] != NULL)
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

    printf("initialising ...\n");

    SetUpSources();

    main_engine_ = new ITMMainEngine(internal_settings_, &image_source_->calib,
                                     image_source_->getRGBImageSize(),
                                     image_source_->getDepthImageSize());

    if (image_source_ == NULL) {
      std::cout << "failed to open any image stream" << std::endl;
    }

    image_source_->main_engine_ = main_engine_;
    pose_source_->main_engine_ = main_engine_;

    UIEngine::Instance()->Initialise(argc, argv, image_source_, imu_source_,
                                     main_engine_, "./Files/Out",
                                     internal_settings_->deviceType);

    // Start already with processing once the run method is called.
    UIEngine::Instance()->mainLoopAction = UIEngine::PROCESS_VIDEO;
    ROS_INFO("GUI Engine Initialized.");
    UIEngine::Instance()->Run();
    ROS_INFO("Done.");
    image_source_->set_camera_pose_ = false;
    UIEngine::Instance()->Shutdown();
  }

  // turn off infinitam.
  if (!request.data) {
    UIEngine::Instance()->mainLoopAction = UIEngine::PROCESS_PAUSED;
    UIEngine::Instance()->mainLoopAction = UIEngine::EXIT;
  }
  // TODO(gocarlos): when the service is called, it does not return true until
  // infinitam is stopped.
  // find a solution.
  response.success = true;
  return true;
}

bool InfinitamNode::publishMap(std_srvs::Empty::Request& request,
                               std_srvs::Empty::Response& response) {

  //////////////////////////////////////////////////////////
  main_engine_->GetMeshingEngine()->MeshScene(main_engine_->GetMesh(),
                                              main_engine_->GetScene());
  main_engine_->GetMesh()->WriteSTL("../output.stl");
  //////////////////////////////////////////////////////////

//  // Publish point cloud.
//  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_pcl(
//      new pcl::PointCloud<pcl::PointXYZ>);
//
//  extractMeshToPcl(point_cloud_pcl);
//  ROS_INFO("got point cloud");
//
//  sensor_msgs::PointCloud2 point_cloud_msg;
//  pcl::toROSMsg(*point_cloud_pcl, point_cloud_msg);
//  point_cloud_msg.header.frame_id = camera_frame_id_;
//  point_cloud_msg.header.stamp = ros::Time::now();
//
//  complete_point_cloud_pub_.publish(point_cloud_msg);
//
//  // Publish ROS mesh
//  shape_msgs::Mesh::Ptr ros_mesh(new shape_msgs::Mesh);
//  extractMeshToRosMesh(ros_mesh);
//  ROS_INFO("got ros mesh");
//
//  complete_mesh_pub_.publish(ros_mesh);

  return true;
}

void InfinitamNode::extractMeshToPcl(
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_pcl) {
  CHECK_NOTNULL(main_engine_);
  CHECK_NOTNULL(&point_cloud_pcl);
  ROS_INFO("extractMeshToPcl");

  main_engine_->GetMeshingEngine()->MeshScene(main_engine_->GetMesh(),
                                              main_engine_->GetScene());
  ROS_INFO("got infinitam mesh successfully");
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

  // Here the point cloud copies the mesh point to the pcl point cloud.
  // That means that the pcl point cloud has some duplicate points.
  point_cloud_pcl->width = main_engine_->GetMesh()->noTotalTriangles * 3;
  point_cloud_pcl->height = 1;
  point_cloud_pcl->is_dense = false;
  point_cloud_pcl->points.resize(
      point_cloud_pcl->width * point_cloud_pcl->height);

  ROS_ERROR_COND(main_engine_->GetMesh()->noTotalTriangles < 1,
                 "The mesh has too few triangles, only: %d",
                 main_engine_->GetMesh()->noTotalTriangles);

  long point_number = 0;
  // All vertices of the mesh are stored in the pcl point cloud.
  for (int64 i = 0; i < main_engine_->GetMesh()->noTotalTriangles; ++i) {
    point_cloud_pcl->points[point_number].x = triangleArray[i].p0.x;
    point_cloud_pcl->points[point_number].y = triangleArray[i].p0.y;
    point_cloud_pcl->points[point_number].z = triangleArray[i].p0.z;
    point_number++;
    point_cloud_pcl->points[point_number].x = triangleArray[i].p1.x;
    point_cloud_pcl->points[point_number].y = triangleArray[i].p1.y;
    point_cloud_pcl->points[point_number].z = triangleArray[i].p1.z;
    point_number++;
    point_cloud_pcl->points[point_number].x = triangleArray[i].p2.x;
    point_cloud_pcl->points[point_number].y = triangleArray[i].p2.y;
    point_cloud_pcl->points[point_number].z = triangleArray[i].p2.z;
    point_number++;
  }
  ROS_INFO("got out_cloud");

  if (rm_triangle_in_cuda_memory) {
    delete cpu_triangles;
  }
}

void InfinitamNode::extractMeshToRosMesh(shape_msgs::Mesh::Ptr ros_mesh) {
  CHECK_NOTNULL(main_engine_);
  CHECK_NOTNULL(&ros_mesh);
  ROS_INFO("extractMeshToRosMesh");

  main_engine_->GetMeshingEngine()->MeshScene(main_engine_->GetMesh(),
                                              main_engine_->GetScene());
  ROS_INFO("got infinitam mesh successfully");
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
//  ROS_INFO("asdf");

  ROS_ERROR_COND(main_engine_->GetMesh()->noTotalTriangles < 1,
                 "The mesh has too few triangles, only: %d",
                 main_engine_->GetMesh()->noTotalTriangles);

  shape_msgs::MeshTriangle ros_triangle;
  geometry_msgs::Point vertices;
  //  ROS_INFO("asdf");

  long index = 0;
  // All vertices of the infinitam mesh are stored in a ROS Mesh.
  for (long i = 0; i < main_engine_->GetMesh()->noTotalTriangles; ++i) {
//    ROS_INFO_STREAM("asdf" << i);

    vertices.x = triangleArray[i].p0.x;
    vertices.y = triangleArray[i].p0.y;
    vertices.z = triangleArray[i].p0.z;
    ros_mesh->vertices.push_back(vertices);
    ros_triangle.vertex_indices[0] = index++;  // 3 - pow(3, i);

    vertices.x = triangleArray[i].p1.x;
    vertices.y = triangleArray[i].p1.y;
    vertices.z = triangleArray[i].p1.z;
    ros_mesh->vertices.push_back(vertices);
    ros_triangle.vertex_indices[1] = index++;  // 2 - pow(3, i);

    vertices.x = triangleArray[i].p2.x;
    vertices.y = triangleArray[i].p2.y;
    vertices.z = triangleArray[i].p2.z;
    ros_mesh->vertices.push_back(vertices);
    ros_triangle.vertex_indices[2] = index++;  // 1 - pow(3, i);

    ros_mesh->triangles.push_back(ros_triangle);
  }
  ROS_INFO_STREAM(
      "ROS mesh has " << ros_mesh->triangles.size() << " triangles, " << "and " << ros_mesh->vertices.size() << " vertices.");
  //  ros_mesh->mergeVertices(0.05);
  //  ROS_INFO("got ros mesh");

  if (rm_triangle_in_cuda_memory) {
    delete cpu_triangles;
  }
}

void InfinitamNode::readParameters() {
  // ROS topic names
  node_handle_.param<std::string>("rgb_image_topic", rgb_image_topic,
                                  "/camera/rgb/image_raw");
  node_handle_.param<std::string>("depth_image_topic", depth_image_topic,
                                  "/camera/depth/image_raw");
  node_handle_.param<std::string>("complete_cloud", complete_cloud_topic_,
                                  "/complete_cloud");
  node_handle_.param<std::string>("complete_mesh", complete_mesh_topic_,
                                  "/complete_mesh");

  // InfiniTAM settings
  node_handle_.param<float>("viewFrustum_min",
                            internal_settings_->sceneParams.viewFrustum_min,
                            0.35f);
  node_handle_.param<float>("viewFrustum_max",
                            internal_settings_->sceneParams.viewFrustum_max,
                            3.0f);

  node_handle_.param<std::string>("camera_frame_id", camera_frame_id_,
                                  "sr300_depth_optical_frame");
}

void InfinitamNode::SetUpSources() {
  CHECK_NOTNULL(pose_source_);
  CHECK_NOTNULL(internal_settings_);

  const char* calibration_filename = arg1;
  const char* depth_image_filename = arg2;
  const char* rgb_image_filename = arg3;
  const char* filename_imu = arg4;

  printf("using calibration file: %s\n", calibration_filename);

  if (rgb_image_filename != NULL) {
    printf("using rgb images: %s\nusing depth images: %s\n",
           depth_image_filename, rgb_image_filename);
    if (filename_imu == NULL) {
      image_source_ = new ImageFileReader(calibration_filename,
                                          depth_image_filename,
                                          rgb_image_filename);
    } else {
      printf("using imu data: %s\n", filename_imu);
      image_source_ = new RawFileReader(calibration_filename,
                                        depth_image_filename,
                                        rgb_image_filename, Vector2i(320, 240),
                                        0.5f);
      imu_source_ = new IMUSourceEngine(filename_imu);
    }
  }

  if (image_source_ == NULL) {
    printf(
        "trying OpenNI device: %s\n",
        (depth_image_filename == NULL) ?
            "<OpenNI default device>" : depth_image_filename);
    image_source_ = new OpenNIEngine(calibration_filename,
                                     depth_image_filename);
    if (image_source_->getDepthImageSize().x == 0) {
      delete image_source_;
      image_source_ = NULL;
    }
  }
  if (image_source_ == NULL) {
    printf("trying UVC device\n");
    image_source_ = new LibUVCEngine(calibration_filename);
    if (image_source_->getDepthImageSize().x == 0) {
      delete image_source_;
      image_source_ = NULL;
    }
  }

  if (image_source_ == NULL) {
    printf("trying MS Kinect 2 device\n");
    image_source_ = new Kinect2Engine(calibration_filename);
    if (image_source_->getDepthImageSize().x == 0) {
      delete image_source_;
      image_source_ = NULL;
    }
  }
  if (image_source_ == NULL) {
    printf("Checking if there are suitable ROS messages being published.\n");

    pose_source_ = new RosPoseSourceEngine(node_handle_);
    image_source_ = new RosImageSourceEngine(node_handle_,
                                             calibration_filename);

    // Get images from ROS topic.
    rgb_sub_ = node_handle_.subscribe(rgb_image_topic, 10,
                                      &RosImageSourceEngine::rgbCallback,
                                      (RosImageSourceEngine*) image_source_);

    depth_sub_ = node_handle_.subscribe(depth_image_topic, 10,
                                        &RosImageSourceEngine::depthCallback,
                                        (RosImageSourceEngine*) image_source_);

    tf_sub_ = node_handle_.subscribe("/tf", 10,
                                     &RosPoseSourceEngine::TFCallback,
                                     (RosPoseSourceEngine*) pose_source_);

    if (image_source_->getDepthImageSize().x == 0) {
      delete image_source_;
      image_source_ = NULL;
    }
  }

  // this is a hack to ensure backwards compatibility in certain configurations
  if (image_source_ == NULL) {
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
