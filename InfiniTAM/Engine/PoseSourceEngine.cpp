// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "PoseSourceEngine.h"

#include <stdio.h>

#include "../Utils/FileUtils.h"

namespace InfiniTAM {
namespace Engine {
PoseSourceEngine::PoseSourceEngine() {
  currentFrameNo = 0;
  cachedFrameNo = -1;

  cached_pose = NULL;
}

void PoseSourceEngine::loadPoseIntoCache(void) {
  // TODO(gocarlos): Implement this
}

bool PoseSourceEngine::hasMoreMeasurements(void) {
  loadPoseIntoCache();
  return (cached_pose != NULL);
}

void PoseSourceEngine::getMeasurement(ITMPoseMeasurement* pose) {
  bool bUsedCache = false;

  if (cached_pose != NULL) {
    pose->R = cached_pose->R;
    pose->T = cached_pose->T;
    delete cached_pose;
    cached_pose = NULL;
    bUsedCache = true;
  }

  if (!bUsedCache) this->loadPoseIntoCache();

  ++currentFrameNo;
}

}  // Namespace Engine
}  // Namespace InfiniTAM
