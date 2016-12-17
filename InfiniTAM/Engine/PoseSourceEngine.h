// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../ITMLib/Objects/ITMPoseMeasurement.h"

#include "../ITMLib/ITMLib.h"

#include <glog/logging.h>

namespace InfiniTAM {
namespace Engine {
class PoseSourceEngine {
 private:
  static const int BUF_SIZE = 2048;
  char poseMask[BUF_SIZE];

  void loadPoseIntoCache();
  int cachedFrameNo;
  int currentFrameNo;

 protected:
  ITMPoseMeasurement* cached_pose;

 public:
  PoseSourceEngine();
  ~PoseSourceEngine() {}
  ITMMainEngine* main_engine_ = nullptr;
  bool set_camera_pose_;

  bool hasMoreMeasurements(void);
  void getMeasurement(ITMPoseMeasurement* pose);
};
}  // Namespace Engine
}  // Namespace InfiniTAM
