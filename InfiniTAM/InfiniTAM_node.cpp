// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include <cstdlib>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "Engine/ImageSourceEngine.h"
#include "Engine/Kinect2Engine.h"
#include "Engine/LibUVCEngine.h"
#include "Engine/OpenNIEngine.h"
#include "Engine/RealSenseEngine.h"
#include "Engine/RosEngine.h"
#include "Engine/UIEngine.h"

using namespace InfiniTAM::Engine;

/** Create a default source of depth images from a list of command line
    arguments. Typically, @para arg1 would identify the calibration file to
    use, @para arg2 the colour images, @para arg3 the depth images and
    @para arg4 the IMU images. If images are omitted, some live sources will
    be tried.
*/
ros::Subscriber rgb_sub_;
ros::Subscriber depth_sub_;

static void CreateDefaultImageSource(const char* arg1, const char* arg2,
                                     const char* arg3, const char* arg4,
                                     ros::NodeHandle& node_handle,
                                     ImageSourceEngine*& image_source,
                                     IMUSourceEngine*& imu_source) {
  const char* calibration_filename = arg1;
  const char* depth_image_filename = arg2;
  const char* rgb_image_filename = arg3;
  const char* filename_imu = arg4;
  std::string rgb_image_topic;
  std::string depth_image_topic;
  node_handle.param<std::string>("rgb_image_topic", rgb_image_topic,
                                 "/camera/rgb/image_raw");
  node_handle.param<std::string>("depth_image_topic", depth_image_topic,
                                 "/camera/depth/image_raw");

  printf("using calibration file: %s\n", calibration_filename);

  if (rgb_image_filename != NULL) {
    printf("using rgb images: %s\nusing depth images: %s\n",
           depth_image_filename, rgb_image_filename);
    if (filename_imu == NULL) {
      image_source = new ImageFileReader(
          calibration_filename, depth_image_filename, rgb_image_filename);
    } else {
      printf("using imu data: %s\n", filename_imu);
      image_source =
          new RawFileReader(calibration_filename, depth_image_filename,
                            rgb_image_filename, Vector2i(320, 240), 0.5f);
      imu_source = new IMUSourceEngine(filename_imu);
    }
  }

  if (image_source == NULL) {
    printf("trying OpenNI device: %s\n", (depth_image_filename == NULL)
                                             ? "<OpenNI default device>"
                                             : depth_image_filename);
    image_source = new OpenNIEngine(calibration_filename, depth_image_filename);
    if (image_source->getDepthImageSize().x == 0) {
      delete image_source;
      image_source = NULL;
    }
  }
  if (image_source == NULL) {
    printf("trying UVC device\n");
    image_source = new LibUVCEngine(calibration_filename);
    if (image_source->getDepthImageSize().x == 0) {
      delete image_source;
      image_source = NULL;
    }
  }
  // TODO(ffurrer): We should maybe consider making this selection a parameter.
  // This is currently commented, as this would get heigher priority before the
  // ROS engine, if a RealSense device is connected.
  // if (image_source == NULL) {
  //   printf("trying RealSense device\n");
  //   image_source = new RealSenseEngine(calibration_filename);
  //   if (image_source->getDepthImageSize().x == 0) {
  //     delete image_source;
  //     image_source = NULL;
  //   }
  // }
  if (image_source == NULL) {
    printf("trying MS Kinect 2 device\n");
    image_source = new Kinect2Engine(calibration_filename);
    if (image_source->getDepthImageSize().x == 0) {
      delete image_source;
      image_source = NULL;
    }
  }
  if (image_source == NULL) {
    printf("Checking if there are suitable ROS messages being published.\n");
    image_source = new RosEngine(node_handle, calibration_filename);

    // Get images from ROS topic.
    rgb_sub_ = node_handle.subscribe(
        rgb_image_topic, 10, &RosEngine::rgbCallback, (RosEngine*)image_source);

    depth_sub_ =
        node_handle.subscribe(depth_image_topic, 10, &RosEngine::depthCallback,
                              (RosEngine*)image_source);

    if (image_source->getDepthImageSize().x == 0) {
      delete image_source;
      image_source = NULL;
    }
  }

  // this is a hack to ensure backwards compatibility in certain configurations
  if (image_source == NULL) {
    return;
  }
  if (image_source->calib.disparityCalib.params == Vector2f(0.0f, 0.0f)) {
    image_source->calib.disparityCalib.type = ITMDisparityCalib::TRAFO_AFFINE;
    image_source->calib.disparityCalib.params = Vector2f(1.0f / 1000.0f, 0.0f);
  }
}

int main(int argc, char** argv) try {
  ros::init(argc, argv, "infinitam_node");
  ros::NodeHandle node_handle("~");

  const char* arg1 = "";
  const char* arg2 = NULL;
  const char* arg3 = NULL;
  const char* arg4 = NULL;

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
  ImageSourceEngine* image_source = NULL;
  IMUSourceEngine* imu_source = NULL;

  CreateDefaultImageSource(arg1, arg2, arg3, arg4, node_handle, image_source,
                           imu_source);
  if (image_source == NULL) {
    std::cout << "failed to open any image stream" << std::endl;
    return -1;
  }

  ITMLibSettings* internal_settings = new ITMLibSettings();
  ITMMainEngine* main_engine = new ITMMainEngine(
      internal_settings, &image_source->calib, image_source->getRGBImageSize(),
      image_source->getDepthImageSize());



  UIEngine::Instance()->Initialise(argc, argv, image_source, imu_source,
                                   main_engine, "./Files/Out",
                                   internal_settings->deviceType);


  ROS_INFO("Initialized.");
  UIEngine::Instance()->Run();
  ROS_INFO("Done.");
  UIEngine::Instance()->Shutdown();

  delete main_engine;
  delete internal_settings;
  delete image_source;
  if (imu_source != NULL) delete imu_source;
  return 0;
} catch (std::exception& e) {
  std::cerr << e.what() << '\n';
  return EXIT_FAILURE;
}