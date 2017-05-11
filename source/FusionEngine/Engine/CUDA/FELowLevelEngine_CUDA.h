// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM
#ifndef _FE_LOWLEVELENGINE_CUDA_H
#define _FE_LOWLEVELENGINE_CUDA_H

#include "../FELowLevelEngine.h"

namespace FE
{
	class FELowLevelEngine_CUDA : public FELowLevelEngine
	{
	public:
		void CopyImage(UChar4Image *image_out, const UChar4Image *image_in) const;
		void CopyImage(FloatImage *image_out, const FloatImage *image_in) const;
		void CopyImage(Float4Image *image_out, const Float4Image *image_in) const;

		void FilterSubsample(UChar4Image *image_out, const UChar4Image *image_in) const;
		void FilterSubsampleWithHoles(FloatImage *image_out, const FloatImage *image_in) const;
		void FilterSubsampleWithHoles(Float4Image *image_out, const Float4Image *image_in) const;

		void GradientX(Short4Image *grad_out, const UChar4Image *image_in) const;
		void GradientY(Short4Image *grad_out, const UChar4Image *image_in) const;

		FELowLevelEngine_CUDA(void);
		~FELowLevelEngine_CUDA(void);
	};
}
#endif //_FE_LOWLEVELENGINE_H
