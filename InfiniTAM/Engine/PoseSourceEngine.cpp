// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "PoseSourceEngine.h"

#include "../Utils/FileUtils.h"
#include <glog/logging.h>

#include <stdio.h>

namespace InfiniTAM {
namespace Engine {
PoseSourceEngine::PoseSourceEngine() {
  currentFrameNo = 0;
  cachedFrameNo = -1;

  cached_pose = NULL;
  LOG(INFO) << "PoseSourceEngine \n";

}

void PoseSourceEngine::loadPoseIntoCache(void) {
  // TODO(gocarlos): Implement this
  LOG(INFO) << "loadPoseIntoCache \n";
}

bool PoseSourceEngine::hasMoreMeasurements(void) {
  loadPoseIntoCache();
  LOG(INFO) << "hasMoreMeasurements \n";

  return (cached_pose != NULL);
}

void PoseSourceEngine::getMeasurement(ITMPoseMeasurement* pose) {
  bool bUsedCache = false;
  LOG(INFO) << "getMeasurement \n";

  if (cached_pose != NULL) {
    pose->R = cached_pose->R;
    pose->T= cached_pose->T;
    delete cached_pose;
    cached_pose = NULL;
    bUsedCache = true;
  }

  if (!bUsedCache) this->loadPoseIntoCache();

  ++currentFrameNo;
}

}  // Namespace Engine
}  // Namespace InfiniTAM
