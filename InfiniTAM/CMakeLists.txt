cmake_minimum_required(VERSION 2.8)
project(infinitam)

add_definitions(-std=c++11)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

IF(WITH_CUDA)
    list(APPEND CUDA_NVCC_FLAGS "-std=c++11")
ELSE()
    SET(CUDA_PROPAGATE_HOST_FLAGS OFF)
ENDIF()

macro(CANONIFY_BOOL var)
  if (${var})
    set(${var} TRUE)
  else()
    set(${var} FALSE)
  endif()
endmacro()

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

find_package(CUDA QUIET)
CANONIFY_BOOL(CUDA_FOUND)
message(STATUS "CUDA found: " ${CUDA_FOUND})

find_package(libuvc QUIET)
CANONIFY_BOOL(libuvc_FOUND)
message(STATUS "libuvc found: ${libuvc_FOUND}")

find_package(librealsense QUIET)
CANONIFY_BOOL(librealsense_FOUND)
message(STATUS "librealsense found: ${librealsense_FOUND}")

find_package(PNG QUIET)
CANONIFY_BOOL(PNG_FOUND)
message(STATUS "libpng found: ${PNG_FOUND}")

OPTION(WITH_CUDA "Build with CUDA support?" ${CUDA_FOUND})
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
OPTION(WITH_REALSENSE "Build with Intel RealSense support?" ${REALSENSE_FOUND})

OPTION(WITH_ROS "Build with ROS support?" TRUE)

IF(MSVC_IDE)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
  add_definitions(-DUSING_CMAKE=1)
ELSE(MSVC_IDE)
  set(CFLAGS_WARN "-Wall -Wextra -Wno-unused-parameter -Wno-strict-aliasing")
  set(CMAKE_CXX_FLAGS "-fPIC -O3 -march=native ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
  #set(CMAKE_CXX_FLAGS "-fPIC -g ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
ENDIF(MSVC_IDE)

if(APPLE)
  set(CUDA_HOST_COMPILER /usr/bin/clang)
endif(APPLE)

IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++ -Wno-deprecated-declarations -Wno-unused-function")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wno-unused-function")
  # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  # set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libstdc++")
ENDIF()

add_subdirectory(ORUtils)
add_subdirectory(Utils)
add_subdirectory(ITMLib)

IF(WITH_CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
ELSE()
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
ENDIF()



find_package(OpenNI QUIET)
CANONIFY_BOOL(OPENNI_FOUND)
message(INFO "OpenNI found: " ${OPENNI_FOUND})

add_subdirectory(Engine)

add_executable(InfiniTAM_cli InfiniTAM_cli.cpp)
target_link_libraries(InfiniTAM_cli Engine Utils)
IF(WITH_CUDA)
  target_link_libraries(InfiniTAM_cli ${CUDA_LIBRARIES})
ENDIF()
add_executable(InfiniTAM InfiniTAM.cpp)
target_link_libraries(InfiniTAM Engine Utils)
IF(WITH_CUDA)
  target_link_libraries(InfiniTAM ${CUDA_LIBRARIES})
ENDIF()

cs_add_executable(infinitam_ros_node infinitam_ros_node.cpp)
IF(WITH_CUDA)
  target_link_libraries(infinitam_ros_node Engine Utils ${CUDA_LIBRARIES})
ELSE()
  target_link_libraries(infinitam_ros_node Engine Utils)
ENDIF()