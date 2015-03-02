// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMViewBuilder_CPU.h"

#include "../../DeviceAgnostic/ITMViewBuilder.h"
#include "../../../../ORUtils/MetalContext.h"

using namespace ITMLib::Engine;
using namespace ORUtils;

ITMViewBuilder_CPU::ITMViewBuilder_CPU(const ITMRGBDCalib *calib):ITMViewBuilder(calib) { }
ITMViewBuilder_CPU::~ITMViewBuilder_CPU(void) { }

void ITMViewBuilder_CPU::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{ 
	if (*view_ptr == NULL)
	{
		*view_ptr = new ITMView(calib, rgbImage->noDims, rawDepthImage->noDims, false);
		if (this->shortImage != NULL) delete this->shortImage;
		this->shortImage = new ITMShortImage(rawDepthImage->noDims, true, false);
		if (this->floatImage != NULL) delete this->floatImage;
		this->floatImage = new ITMFloatImage(rawDepthImage->noDims, true, false);

	}
	ITMView *view = *view_ptr;

	view->rgb->SetFrom(rgbImage, MemoryBlock<Vector4u>::CPU_TO_CPU);
	this->shortImage->SetFrom(rawDepthImage, MemoryBlock<short>::CPU_TO_CPU);

	switch (inputImageType)
	{
	case InfiniTAM_DISPARITY_IMAGE:
		this->ConvertDisparityToDepth(this->floatImage, this->shortImage, &(view->calib->intrinsics_d), &(view->calib->disparityCalib));
		break;
	case InfiniTAM_SHORT_DEPTH_IMAGE:
		this->ConvertDepthMMToFloat(this->floatImage, this->shortImage);
		break;
	default:
		break;
	}

	view->depth->SetFrom(this->floatImage, MemoryBlock<float>::CPU_TO_CPU);
	//this->SmoothRawDepth(view->depth, this->floatImage);
}

void ITMViewBuilder_CPU::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMFloatImage *depthImage)
{
	if (*view_ptr == NULL)
	{
		*view_ptr = new ITMView(calib, rgbImage->noDims, depthImage->noDims, false);
	}
	ITMView *view = *view_ptr;

	view->rgb->UpdateDeviceFromHost();
	view->depth->UpdateDeviceFromHost();
}

void ITMViewBuilder_CPU::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMShortImage *depthImage, ITMIMUMeasurement *imuMeasurement)
{
	if (*view_ptr == NULL)
	{
		*view_ptr = new ITMViewIMU(calib, rgbImage->noDims, depthImage->noDims, false);
		if (this->shortImage != NULL) delete this->shortImage;
		this->shortImage = new ITMShortImage(depthImage->noDims, true, false);
	}
	ITMViewIMU* imuView = (ITMViewIMU*)(*view_ptr);
	imuView->imu->SetFrom(imuMeasurement);

	this->UpdateView(view_ptr, rgbImage, depthImage);
}

void ITMViewBuilder_CPU::ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *depth_in, const ITMIntrinsics *depthIntrinsics,
	const ITMDisparityCalib *disparityCalib)
{
	Vector2i imgSize = depth_in->noDims;

	const short *d_in = depth_in->GetData(MEMORYDEVICE_CPU);
	float *d_out = depth_out->GetData(MEMORYDEVICE_CPU);

	Vector2f disparityCalibParams; float fx_depth;
	disparityCalibParams.x = disparityCalib->params.x;
	disparityCalibParams.y = disparityCalib->params.y;
	fx_depth = depthIntrinsics->projectionParamsSimple.fx;

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
		convertDisparityToDepth(d_out, x, y, d_in, disparityCalibParams, fx_depth, imgSize);
}

void ITMViewBuilder_CPU::ConvertDepthMMToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in)
{
	Vector2i imgSize = depth_in->noDims;

	const short *d_in = depth_in->GetData(MEMORYDEVICE_CPU);
	float *d_out = depth_out->GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
		convertDepthMMToFloat(d_out, x, y, d_in, imgSize);
}

void ITMLib::Engine::ITMViewBuilder_CPU::SmoothRawDepth(ITMFloatImage *image_out, const ITMFloatImage *image_in)
{
	Vector2i imgSize = image_in->noDims;

	float *imout = image_out->GetData(MEMORYDEVICE_CPU);
	const float *imin = image_in->GetData(MEMORYDEVICE_CPU);

	memset(imout, 0, imgSize.x * imgSize.y * sizeof(float));

	for (int y = 1; y < imgSize.y - 1; y++) for (int x = 1; x < imgSize.x - 1; x++)
		smoothingRawDepth(imout, imin, x, y, imgSize);
}
