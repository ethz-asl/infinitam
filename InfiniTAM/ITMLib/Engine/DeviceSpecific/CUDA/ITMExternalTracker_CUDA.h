// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMExternalTracker.h"

namespace ITMLib {
namespace Engine {
class ITMExternalTracker_CUDA : public ITMExternalTracker {
 private:
  Vector2f* f_device;
  float *g_device, *h_device;
  Vector2f* f_host;
  float *g_host, *h_host;

 public:
  void F_oneLevel(float* f, ITMPose* pose);
  void G_oneLevel(float* gradient, float* hessian, ITMPose* pose) const;

  ITMExternalTracker_CUDA(Vector2i imgSize,
                          TrackerIterationType* trackingRegime,
                          int noHierarchyLevels,
                          const ITMLowLevelEngine* lowLevelEngine);
  ~ITMExternalTracker_CUDA(void);
};
}
}
