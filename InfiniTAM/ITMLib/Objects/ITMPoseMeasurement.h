// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

namespace ITMLib {
namespace Objects {
class ITMPoseMeasurement {
 public:
  Matrix3f R;
  Vector3f T;

  ITMPoseMeasurement() {
    this->R.setIdentity();
    this->T.x = 0;
    this->T.y = 0;
    this->T.z = 0;
  }

  ITMPoseMeasurement(const Matrix3f& R, const Vector3f T) {
    this->R = R;
    this->T = T;
  }

  void SetFrom(const ITMPoseMeasurement* measurement) {
    this->R = measurement->R;
    this->T = measurement->T;
  }

  ~ITMPoseMeasurement(void) {}

  // Suppress the default copy constructor and assignment operator
  ITMPoseMeasurement(const ITMPoseMeasurement&);
  ITMPoseMeasurement& operator=(const ITMPoseMeasurement&);
};
}  // namespace Objects
}  // namespace ITMLib
