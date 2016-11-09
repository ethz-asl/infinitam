// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMExternalTracker.h"

namespace ITMLib {
namespace Engine {
class ITMExternalTracker_CPU : public ITMExternalTracker {
 public:
  void F_oneLevel(float* f, ITMPose* pose);
  void G_oneLevel(float* gradient, float* hessian, ITMPose* pose) const;

  ITMExternalTracker_CPU(Vector2i imgSize, TrackerIterationType* trackingRegime,
                         int noHierarchyLevels,
                         const ITMLowLevelEngine* lowLevelEngine);
  ~ITMExternalTracker_CPU(void);
};
}
}
