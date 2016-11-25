// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "PoseSourceEngine.h"

#include "../Utils/FileUtils.h"

#include <stdio.h>

using namespace InfiniTAM::Engine;

PoseSourceEngine::PoseSourceEngine(const char* poseMask) {
  strncpy(this->poseMask, poseMask, BUF_SIZE);

  currentFrameNo = 0;
  cachedFrameNo = -1;

  cached_pose = NULL;
}

void PoseSourceEngine::loadPoseIntoCache(void) {
  char str[2048];
  FILE* f;
  bool success = false;

  cached_pose = new ITMPose();

  sprintf(str, poseMask, currentFrameNo);
  f = fopen(str, "r");
  if (f) {
    size_t ret =
        fscanf(f, "%f %f %f %f %f %f %f", /*TODO(gocarlos): Implement this*/);

    fclose(f);

    if (ret == 9) success = true;
  }

  if (!success) {
    delete cached_pose;
    cached_pose = NULL;
    printf("error reading file '%s'\n", str);
  }
}

bool PoseSourceEngine::hasMoreMeasurements(void) {
  loadPoseIntoCache();

  return (cached_pose != NULL);
}

void PoseSourceEngine::getMeasurement(ITMPoseMeasurement* pose) {
  bool bUsedCache = false;

  if (cached_pose != NULL) {
    pose->R = cached_pose->R;
    delete cached_pose;
    cached_pose = NULL;
    bUsedCache = true;
  }

  if (!bUsedCache) this->loadPoseIntoCache();

  ++currentFrameNo;
}
