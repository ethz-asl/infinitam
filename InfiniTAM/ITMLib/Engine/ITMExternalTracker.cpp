// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMExternalTracker.h"

#include <glog/logging.h>
#include <math.h>

#include "../../ORUtils/Cholesky.h"


using namespace ITMLib::Engine;

ITMExternalTracker::ITMExternalTracker(Vector2i imgSize,
                                       TrackerIterationType* trackingRegime,
                                       int noHierarchyLevels,
                                       const ITMLowLevelEngine* lowLevelEngine,
                                       MemoryDeviceType memoryType) {
  viewHierarchy = new ITMImageHierarchy<ITMViewHierarchyLevel>(
      imgSize, trackingRegime, noHierarchyLevels, memoryType);

  this->lowLevelEngine = lowLevelEngine;
  LOG(INFO)<<"ITMExternalTracker";
}

ITMExternalTracker::~ITMExternalTracker(void) {
  delete viewHierarchy;
}

void ITMExternalTracker::TrackCamera(ITMTrackingState* trackingState,
                                     const ITMView* view) {
  //TODO(gocarlos): merge information from the camera pose with ICP estimation here.
}
