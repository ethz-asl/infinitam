// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMExternalTracker_CPU.h"


#include "../../DeviceAgnostic/ITMExternalTracker.h"
#include "../../DeviceAgnostic/ITMPixelUtils.h"
#include "../../ITMExternalTracker.h"

using namespace ITMLib::Engine;

ITMExternalTracker_CPU::ITMExternalTracker_CPU(
    Vector2i imgSize, TrackerIterationType* trackingRegime,
    int noHierarchyLevels, const ITMLowLevelEngine* lowLevelEngine)
    : ITMExternalTracker(imgSize, trackingRegime, noHierarchyLevels,
                         lowLevelEngine, MEMORYDEVICE_CPU) {}

ITMExternalTracker_CPU::~ITMExternalTracker_CPU(void) {}

void ITMExternalTracker_CPU::F_oneLevel(float* f, ITMPose* pose) {
  LOG(FATAL) << "not implemented yet";
}

void ITMExternalTracker_CPU::G_oneLevel(float* gradient, float* hessian,
                                        ITMPose* pose) const {
  LOG(FATAL) << "not implemented yet";
}
