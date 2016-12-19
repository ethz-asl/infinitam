// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#include "../Objects/ITMImageHierarchy.h"
#include "../Objects/ITMViewHierarchyLevel.h"

#include "../Engine/ITMLowLevelEngine.h"
#include "../Engine/ITMTracker.h"

using namespace ITMLib::Objects;

namespace ITMLib {
namespace Engine {
/** Base class for engines using an external tracking system (e.g. Vicon).
   Implementations would typically take the odometry or pose measurements from
   the external tracking system and get the current view frame from it.
*/
class ITMExternalTracker : public ITMTracker {
 private:
  const ITMLowLevelEngine* lowLevelEngine;

  void PrepareForEvaluation(const ITMView* view);
 protected:
  TrackerIterationType iterationType;

  ITMTrackingState* trackingState;

  const ITMView* view;
  ITMImageHierarchy<ITMViewHierarchyLevel>* viewHierarchy;
  int levelId;
 public:
  void TrackCamera(ITMTrackingState* trackingState, const ITMView* view);

  ITMExternalTracker(Vector2i imgSize, TrackerIterationType* trackingRegime,
                     int noHierarchyLevels,
                     const ITMLowLevelEngine* lowLevelEngine,
                     MemoryDeviceType memoryType);
  virtual ~ITMExternalTracker(void);
};
} // namespace Engine
} // namespace ITMLib
