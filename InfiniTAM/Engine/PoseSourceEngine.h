// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../ITMLib/ITMLib.h"

#include "../ITMLib/Objects/ITMPoseMeasurement.h"

namespace InfiniTAM {
namespace Engine {
class PoseSourceEngine {
 private:
  static const int BUF_SIZE = 2048;
  char poseMask[BUF_SIZE];

  ITMPoseMeasurement* cached_pose;

  void loadPoseIntoCache();
  int cachedFrameNo;
  int currentFrameNo;

 public:
  PoseSourceEngine();
  ~PoseSourceEngine() {
  }

  bool hasMoreMeasurements(void);
  void getMeasurement(ITMPoseMeasurement* pose);
};
}  // Namespace Engine
}  // Namespace InfiniTAM
