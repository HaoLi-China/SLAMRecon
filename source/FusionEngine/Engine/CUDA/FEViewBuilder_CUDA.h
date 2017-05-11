// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM.
#ifndef _FE_VIEWBUILDER_CUDA_H
#define _FE_VIEWBUILDER_CUDA_H

#include "../FEViewBuilder.h"

namespace FE
{
	class FEViewBuilder_CUDA : public FEViewBuilder
	{
	public:
		void ConvertDisparityToDepth(FloatImage *depth_out, const ShortImage *depth_in, const FEIntrinsics *depthIntrinsics,
			Vector2f disparityCalibParams);
		void ConvertDepthAffineToFloat(FloatImage *depth_out, const ShortImage *depth_in, Vector2f depthCalibParams);

		void DepthFiltering(FloatImage *image_out, const FloatImage *image_in);
		void ComputeNormalAndWeights(Float4Image *normal_out, FloatImage *sigmaZ_out, const FloatImage *depth_in, Vector4f intrinsic);

		void UpdateView(FEView **view, UChar4Image *rgbImage, ShortImage *rawDepthImage, bool useBilateralFilter, bool modelSensorNoise = false);
		void UpdateView(FEView **view, UChar4Image *rgbImage, FloatImage *depthImage);

		FEViewBuilder_CUDA(const FERGBDCalib *calib);
		~FEViewBuilder_CUDA(void);
	};
}
#endif //_FE_VIEWBUILDER_CUDA_H

