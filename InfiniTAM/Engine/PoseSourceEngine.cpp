// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "PoseSourceEngine.h"

#include "../Utils/FileUtils.h"

#include <stdio.h>

namespace InfiniTAM {
namespace Engine {
PoseSourceEngine::PoseSourceEngine() {
//  strncpy(this->poseMask, poseMask, BUF_SIZE);
//
  currentFrameNo = 0;
  cachedFrameNo = -1;

  cached_pose = NULL;
}

void PoseSourceEngine::loadPoseIntoCache(void) {
  // TODO(gocarlos): Implement this

//  char str[2048];
//  FILE* f;
//  bool success = false;
//
//  cached_pose = new ITMPoseMeasurement();
//
//  sprintf(str, poseMask, currentFrameNo);
//  f = fopen(str, "r");
//  if (f) {
//    size_t ret =0; //        fscanf(f, "%f %f %f %f %f %f %f");
//
//    fclose(f);
//
//    if (ret == 9) success = true;
//  }
//
//  if (!success) {
//    delete cached_pose;
//    cached_pose = NULL;
//    printf("error reading file '%s'\n", str);
//  }
//
}

bool PoseSourceEngine::hasMoreMeasurements(void) {
  loadPoseIntoCache();

  return (cached_pose != NULL);
}

void PoseSourceEngine::getMeasurement(ITMPoseMeasurement* pose) {
  bool bUsedCache = false;

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
