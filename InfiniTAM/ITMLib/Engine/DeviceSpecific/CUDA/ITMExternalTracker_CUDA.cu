// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMExternalTracker_CUDA.h"

#include <glog/logging.h>

#include "../../DeviceAgnostic/ITMExternalTracker.h"
#include "../../DeviceAgnostic/ITMPixelUtils.h"
#include "ITMCUDAUtils.h"

using namespace ITMLib::Engine;

ITMExternalTracker_CUDA::ITMExternalTracker_CUDA(
    Vector2i imgSize, TrackerIterationType* trackingRegime,
    int noHierarchyLevels, const ITMLowLevelEngine* lowLevelEngine)
    : ITMExternalTracker(imgSize, trackingRegime, noHierarchyLevels,
                         lowLevelEngine, MEMORYDEVICE_CUDA) {}

ITMExternalTracker_CUDA::~ITMExternalTracker_CUDA(void) {}

void ITMExternalTracker_CUDA::F_oneLevel(float* f, ITMPose* pose) {}

void ITMExternalTracker_CUDA::G_oneLevel(float* gradient, float* hessian,
                                         ITMPose* pose) const {}
