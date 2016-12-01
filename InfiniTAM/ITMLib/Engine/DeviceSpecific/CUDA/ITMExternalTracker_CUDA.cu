// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "../../DeviceAgnostic/ITMExternalTracker.h"
#include "../../DeviceAgnostic/ITMPixelUtils.h"
#include "ITMCUDAUtils.h"
#include "ITMExternalTracker_CUDA.h"
#include <glog/logging.h>

using namespace ITMLib::Engine;

//__global__ void externalTrackerOneLevel_f_device(
//    Vector2f* out, Vector4f* locations, Vector4f* colours, Vector4u* rgb,
//    int noTotalPoints, Matrix4f M, Vector4f projParams, Vector2i imgSize);
//
//__global__ void externalTrackerOneLevel_g_rt_device(
//    float* g_out, float* h_out, Vector4f* locations, Vector4f* colours,
//    Vector4s* gx, Vector4s* gy, Vector4u* rgb, int noTotalPoints, Matrix4f M,
//    Vector4f projParams, Vector2i imgSize);
//__global__ void externalTrackerOneLevel_g_ro_device(
//    float* g_out, float* h_out, Vector4f* locations, Vector4f* colours,
//    Vector4s* gx, Vector4s* gy, Vector4u* rgb, int noTotalPoints, Matrix4f M,
//    Vector4f projParams, Vector2i imgSize);

// host methods

ITMExternalTracker_CUDA::ITMExternalTracker_CUDA(
    Vector2i imgSize, TrackerIterationType* trackingRegime,
    int noHierarchyLevels, const ITMLowLevelEngine* lowLevelEngine)
    : ITMExternalTracker(imgSize, trackingRegime, noHierarchyLevels,
                         lowLevelEngine, MEMORYDEVICE_CUDA) {
  LOG(ERROR) << "not implemented yet";
}

ITMExternalTracker_CUDA::~ITMExternalTracker_CUDA(void) {
  LOG(ERROR) << "not implemented yet";
}

void ITMExternalTracker_CUDA::F_oneLevel(float* f, ITMPose* pose) {
  LOG(ERROR) << "not implemented yet";
}

void ITMExternalTracker_CUDA::G_oneLevel(float* gradient, float* hessian,
                                         ITMPose* pose) const {
  LOG(ERROR) << "not implemented yet";
}
