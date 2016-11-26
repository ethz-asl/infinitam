// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMExternalTracker.h"
#include "../../ORUtils/Cholesky.h"

#include <math.h>

using namespace ITMLib::Engine;

static inline bool minimizeLM(const ITMExternalTracker& tracker,
                              ITMPose& initialization);

ITMExternalTracker::ITMExternalTracker(Vector2i imgSize,
                                       TrackerIterationType* trackingRegime,
                                       int noHierarchyLevels,
                                       const ITMLowLevelEngine* lowLevelEngine,
                                       MemoryDeviceType memoryType) {
  viewHierarchy = new ITMImageHierarchy<ITMViewHierarchyLevel>(
      imgSize, trackingRegime, noHierarchyLevels, memoryType);

  this->lowLevelEngine = lowLevelEngine;
  ROS_INFO("ITMExternalTracker");
}

ITMExternalTracker::~ITMExternalTracker(void) {
//  delete viewHierarchy;
}

void ITMExternalTracker::TrackCamera(ITMTrackingState* trackingState,
                                     const ITMView* view) {
  this->view = view;
  this->trackingState = trackingState;


//  ITMPose currentPara(
//      view->calib->trafo_rgb_to_depth.calib_inv
//          * trackingState->pose_d->GetM());

  // these following will coerce the result back into the chosen
  // parameterization for rotations
//  trackingState->pose_d->SetM(
//      view->calib->trafo_rgb_to_depth.calib * currentPara.GetM());

  trackingState->pose_d->Coerce();
}