include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})

IF(WITH_CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
ELSE()
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
ENDIF()

IF(OPENNI_FOUND)
  include_directories(${OpenNI_INCLUDE_DIR})
ELSE(OPENNI_FOUND)
  add_definitions(-DCOMPILE_WITHOUT_OpenNI)
ENDIF(OPENNI_FOUND)

IF(MSKINECTAPI_FOUND)
  include_directories(${MSKINECTAPI_INCLUDE_DIR})
ELSE(MSKINECTAPI_FOUND)
  add_definitions(-DCOMPILE_WITHOUT_Kinect2)
ENDIF(MSKINECTAPI_FOUND)

IF(DEFINED libuvc_INCLUDE_DIRS)
  include_directories(${libuvc_INCLUDE_DIRS})
  add_definitions(-DCOMPILE_WITH_LibUVC)
ENDIF()

IF(WITH_REALSENSE)
  include_directories(${librealsense_INCLUDE_DIRS})
  add_definitions(-DCOMPILE_WITH_RealSense)
  SET_PROPERTY(SOURCE RealSenseEngine.cpp PROPERTY COMPILE_FLAGS -std=c++11)
ENDIF()

IF(WITH_ROS)
  include_directories(${Ros_INCLUDE_DIR})
  add_definitions(-DCOMPILE_WITH_Ros)
  SET_PROPERTY(SOURCE RosEngine.cpp PROPERTY COMPILE_FLAGS -std=c++11)
ENDIF()

add_library(Engine
CLIEngine.cpp
CLIEngine.h
ImageSourceEngine.cpp
ImageSourceEngine.h
PoseSourceEngine.cpp
PoseSourceEngine.h
IMUSourceEngine.cpp
IMUSourceEngine.h
Kinect2Engine.cpp
Kinect2Engine.h
LibUVCEngine.cpp
LibUVCEngine.h
OpenNIEngine.cpp
OpenNIEngine.h
RealSenseEngine.cpp
RealSenseEngine.h
RosEngine.cpp
RosEngine.h
RosImageSourceEngine.cpp
RosImageSourceEngine.h
RosPoseSourceEngine.cpp
RosPoseSourceEngine.h
UIEngine.cpp
UIEngine.h
)

target_link_libraries(Engine ${GLUT_LIBRARIES})
target_link_libraries(Engine ${OPENGL_LIBRARIES})
target_link_libraries(Engine ITMLib)
IF(WITH_CUDA)
  target_link_libraries(Engine ${CUDA_LIBRARY})
ENDIF()
IF(OPENNI_FOUND)
  target_link_libraries(Engine ${OpenNI_LIBRARY})
ENDIF(OPENNI_FOUND)
IF(DEFINED libuvc_LIBRARIES)
  target_link_libraries(Engine ${libuvc_LIBRARIES})
ENDIF()
IF(WITH_REALSENSE)
  TARGET_LINK_LIBRARIES(Engine ${librealsense_LIBRARIES})
ENDIF()
IF(WITH_ROS)
  TARGET_LINK_LIBRARIES(Engine ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${glog_catkin_LIBRARIES})
ENDIF()
