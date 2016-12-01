// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>

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
  double x, y, z;
  double t, u, v;
 protected:
  TrackerIterationType iterationType;

  // write into the pose
  //  trackingState->pose_d;
  // TODO(gocarlos) change this here, provide a first guess from the ROS TF
  ITMTrackingState* trackingState;

  const ITMView* view;
  ITMImageHierarchy<ITMViewHierarchyLevel>* viewHierarchy;
  int levelId;

  int countedPoints_valid;

 public:
  class EvaluationPoint {
   public:
    float f(void) { return cacheF; }
    const float* nabla_f(void) {
      if (cacheNabla == NULL) computeGradients(false);
      return cacheNabla;
    }

    const float* hessian_GN(void) {
      if (cacheHessian == NULL) computeGradients(true);
      return cacheHessian;
    }
    const ITMPose& getParameter(void) const { return *mPara; }

    EvaluationPoint(ITMPose* pos, const ITMExternalTracker* f_parent);
    ~EvaluationPoint(void) {
      delete mPara;
      if (cacheNabla != NULL) delete[] cacheNabla;
      if (cacheHessian != NULL) delete[] cacheHessian;
    }

   protected:
    void computeGradients(bool requiresHessian);


    ITMPose* mPara;
    const ITMExternalTracker* mParent;

    float cacheF;
    float* cacheNabla;
    float* cacheHessian;
  };

  EvaluationPoint* evaluateAt(ITMPose* para) const {
    return new EvaluationPoint(para, this);
  }



  int numParameters(void) const {
    return (iterationType == TRACKER_ITERATION_ROTATION) ? 3 : 6;
  }

  virtual void F_oneLevel(float* f, ITMPose* pose) = 0;
  virtual void G_oneLevel(float* gradient, float* hessian,
                          ITMPose* pose) const = 0;

  void ApplyDelta(const ITMPose& para_old, const float* delta,
                  ITMPose& para_new) const;

  void TrackCamera(ITMTrackingState* trackingState, const ITMView* view);

  ITMExternalTracker(Vector2i imgSize, TrackerIterationType* trackingRegime,
                     int noHierarchyLevels,
                     const ITMLowLevelEngine* lowLevelEngine,
                     MemoryDeviceType memoryType);
  virtual ~ITMExternalTracker(void);
};
} // namespace Engine
} // namespace ITMLib
