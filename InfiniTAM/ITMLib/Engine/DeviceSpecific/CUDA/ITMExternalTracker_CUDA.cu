// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "../../DeviceAgnostic/ITMExternalTracker.h"
#include "../../DeviceAgnostic/ITMPixelUtils.h"
#include "ITMCUDAUtils.h"
#include "ITMExternalTracker_CUDA.h"

using namespace ITMLib::Engine;

//__global__ void externalTrackerOneLevel_f_device(
//    Vector2f* out, Vector4f* locations, Vector4f* colours, Vector4u* rgb,
//    int noTotalPoints, Matrix4f M, Vector4f projParams, Vector2i imgSize);
//
//__global__ void externalTrackerOneLevel_g_rt_device(
//    float* g_out, float* h_out, Vector4f* locations, Vector4f* colours,
//    Vector4s* gx, Vector4s* gy, Vector4u* rgb, int noTotalPoints, Matrix4f M,
//    Vector4f projParams, Vector2i imgSize);
//__global__ void externalTrackerOneLevel_g_ro_device(
//    float* g_out, float* h_out, Vector4f* locations, Vector4f* colours,
//    Vector4s* gx, Vector4s* gy, Vector4u* rgb, int noTotalPoints, Matrix4f M,
//    Vector4f projParams, Vector2i imgSize);

// host methods

ITMExternalTracker_CUDA::ITMExternalTracker_CUDA(
    Vector2i imgSize, TrackerIterationType* trackingRegime,
    int noHierarchyLevels, const ITMLowLevelEngine* lowLevelEngine)
    : ITMExternalTracker(imgSize, trackingRegime, noHierarchyLevels,
                         lowLevelEngine, MEMORYDEVICE_CUDA) {
//  int dim_g = 6;
//  int dim_h = 6 + 5 + 4 + 3 + 2 + 1;
//
//  ITMSafeCall(cudaMalloc((void**)&f_device,
//                         sizeof(Vector2f) * (imgSize.x * imgSize.y / 128)));
//  ITMSafeCall(cudaMalloc((void**)&g_device, sizeof(float) * dim_g *
//                                                (imgSize.x * imgSize.y / 128)));
//  ITMSafeCall(cudaMalloc((void**)&h_device, sizeof(float) * dim_h *
//                                                (imgSize.x * imgSize.y / 128)));
//
//  f_host = new Vector2f[imgSize.x * imgSize.y / 128];
//  g_host = new float[dim_g * imgSize.x * imgSize.y / 128];
//  h_host = new float[dim_h * imgSize.x * imgSize.y / 128];
}

ITMExternalTracker_CUDA::~ITMExternalTracker_CUDA(void) {
//  ITMSafeCall(cudaFree(f_device));
//  ITMSafeCall(cudaFree(g_device));
//  ITMSafeCall(cudaFree(h_device));
//
//  delete[] f_host;
//  delete[] g_host;
//  delete[] h_host;
}

void ITMExternalTracker_CUDA::F_oneLevel(float* f, ITMPose* pose) {
//  int noTotalPoints = trackingState->pointCloud->noTotalPoints;
//
//  Vector4f projParams = view->calib->intrinsics_rgb.projectionParamsSimple.all;
//  projParams.x /= 1 << levelId;
//  projParams.y /= 1 << levelId;
//  projParams.z /= 1 << levelId;
//  projParams.w /= 1 << levelId;
//
//  Matrix4f M = pose->GetM();
//
//  Vector2i imgSize = viewHierarchy->levels[levelId]->rgb->noDims;
//
//  float scaleForOcclusions, final_f;
//
//  Vector4f* locations =
//      trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CUDA);
//  Vector4f* colours =
//      trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CUDA);
//  Vector4u* rgb =
//      viewHierarchy->levels[levelId]->rgb->GetData(MEMORYDEVICE_CUDA);
//
//  dim3 blockSize(128, 1);
//  dim3 gridSize((int)ceil((float)noTotalPoints / (float)blockSize.x), 1);
//
//  ITMSafeCall(cudaMemset(f_device, 0, sizeof(Vector2f) * gridSize.x));
//
//  externalTrackerOneLevel_f_device<<<gridSize, blockSize>>>(
//      f_device, locations, colours, rgb, noTotalPoints, M, projParams, imgSize);
//
//  ITMSafeCall(cudaMemcpy(f_host, f_device, sizeof(Vector2f) * gridSize.x,
//                         cudaMemcpyDeviceToHost));
//
//  final_f = 0;
//  countedPoints_valid = 0;
//  for (size_t i = 0; i < gridSize.x; i++) {
//    final_f += f_host[i].x;
//    countedPoints_valid += (int)f_host[i].y;
//  }
//
//  if (countedPoints_valid == 0) {
//    final_f = MY_INF;
//    scaleForOcclusions = 1.0;
//  } else {
//    scaleForOcclusions = (float)noTotalPoints / countedPoints_valid;
//  }
//
//  f[0] = final_f * scaleForOcclusions;
}

void ITMExternalTracker_CUDA::G_oneLevel(float* gradient, float* hessian,
                                         ITMPose* pose) const {
//  int noTotalPoints = trackingState->pointCloud->noTotalPoints;
//
//  Vector4f projParams = view->calib->intrinsics_rgb.projectionParamsSimple.all;
//  projParams.x /= 1 << levelId;
//  projParams.y /= 1 << levelId;
//  projParams.z /= 1 << levelId;
//  projParams.w /= 1 << levelId;
//
//  Matrix4f M = pose->GetM();
//
//  Vector2i imgSize = viewHierarchy->levels[levelId]->rgb->noDims;
//
//  float scaleForOcclusions;
//
//  bool rotationOnly = iterationType == TRACKER_ITERATION_ROTATION;
//  int numPara = rotationOnly ? 3 : 6,
//      numParaSQ = rotationOnly ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;
//
//  float globalGradient[6], globalHessian[21];
//  for (int i = 0; i < numPara; i++) globalGradient[i] = 0.0f;
//  for (int i = 0; i < numParaSQ; i++) globalHessian[i] = 0.0f;
//
//  Vector4f* locations =
//      trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CUDA);
//  Vector4f* colours =
//      trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CUDA);
//  Vector4u* rgb =
//      viewHierarchy->levels[levelId]->rgb->GetData(MEMORYDEVICE_CUDA);
//  Vector4s* gx =
//      viewHierarchy->levels[levelId]->gradientX_rgb->GetData(MEMORYDEVICE_CUDA);
//  Vector4s* gy =
//      viewHierarchy->levels[levelId]->gradientY_rgb->GetData(MEMORYDEVICE_CUDA);
//
//  dim3 blockSize(128, 1);
//  dim3 gridSize((int)ceil((float)noTotalPoints / (float)blockSize.x), 1);
//
//  if (rotationOnly) {
//    ITMSafeCall(cudaMemset(g_device, 0, sizeof(float) * gridSize.x * 3));
//    ITMSafeCall(cudaMemset(h_device, 0, sizeof(float) * gridSize.x * 6));
//
//    externalTrackerOneLevel_g_ro_device<<<gridSize, blockSize>>>(
//        g_device, h_device, locations, colours, gx, gy, rgb, noTotalPoints, M,
//        projParams, imgSize);
//  } else {
//    ITMSafeCall(cudaMemset(g_device, 0, sizeof(float) * gridSize.x * 6));
//    ITMSafeCall(cudaMemset(h_device, 0, sizeof(float) * gridSize.x * 21));
//
//    externalTrackerOneLevel_g_rt_device<<<gridSize, blockSize>>>(
//        g_device, h_device, locations, colours, gx, gy, rgb, noTotalPoints, M,
//        projParams, imgSize);
//  }
//
//  ITMSafeCall(cudaMemcpy(g_host, g_device, sizeof(float) * gridSize.x * numPara,
//                         cudaMemcpyDeviceToHost));
//  ITMSafeCall(cudaMemcpy(h_host, h_device,
//                         sizeof(float) * gridSize.x * numParaSQ,
//                         cudaMemcpyDeviceToHost));
//
//  for (size_t i = 0; i < gridSize.x; i++) {
//    for (int p = 0; p < numPara; p++)
//      globalGradient[p] += g_host[i * numPara + p];
//    for (int p = 0; p < numParaSQ; p++)
//      globalHessian[p] += h_host[i * numParaSQ + p];
//  }
//
//  scaleForOcclusions = (float)noTotalPoints / countedPoints_valid;
//  if (countedPoints_valid == 0) {
//    scaleForOcclusions = 1.0f;
//  }
//
//  for (int para = 0, counter = 0; para < numPara; para++) {
//    gradient[para] = globalGradient[para] * scaleForOcclusions;
//    for (int col = 0; col <= para; col++, counter++)
//      hessian[para + col * numPara] =
//          globalHessian[counter] * scaleForOcclusions;
//  }
//  for (int row = 0; row < numPara; row++) {
//    for (int col = row + 1; col < numPara; col++)
//      hessian[row + col * numPara] = hessian[col + row * numPara];
//  }
}
