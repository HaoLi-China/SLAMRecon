// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM.
#ifndef _FE_VIEWBUILDER_H
#define _FE_VIEWBUILDER_H

#include "../Utils/FELibDefines.h"
#include "../Utils/FELibSettings.h"

#include "../Objects/FEView.h"
#include "../Objects/FERGBDCalib.h"

namespace FE
{
	/** \brief
	*/
	class FEViewBuilder
	{
	protected:
		const FERGBDCalib *calib;
		ShortImage *shortImage;
		FloatImage *floatImage;

	public:
		virtual void ConvertDisparityToDepth(FloatImage *depth_out, const ShortImage *disp_in, const FEIntrinsics *depthIntrinsics,
			Vector2f disparityCalibParams) = 0;
		virtual void ConvertDepthAffineToFloat(FloatImage *depth_out, const ShortImage *depth_in, Vector2f depthCalibParams) = 0;

		virtual void DepthFiltering(FloatImage *image_out, const FloatImage *image_in) = 0;
		virtual void ComputeNormalAndWeights(Float4Image *normal_out, FloatImage *sigmaZ_out, const FloatImage *depth_in, Vector4f intrinsic) = 0;

		virtual void UpdateView(FEView **view, UChar4Image *rgbImage, ShortImage *rawDepthImage, bool useBilateralFilter, bool modelSensorNoise = false) = 0;
		virtual void UpdateView(FEView **view, UChar4Image *rgbImage, FloatImage *depthImage) = 0;


		FEViewBuilder(const FERGBDCalib *calib)
		{
			this->calib = calib;
			this->shortImage = NULL;
			this->floatImage = NULL;
		}

		virtual ~FEViewBuilder()
		{
			if (this->shortImage != NULL) delete this->shortImage;
			if (this->floatImage != NULL) delete this->floatImage;
		}
	};
}
#endif //_FE_VIEWBUILDER_H
