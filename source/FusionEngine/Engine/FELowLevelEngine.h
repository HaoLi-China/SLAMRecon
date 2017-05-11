// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM.
#ifndef _FE_LOWLEVELENGINE_H
#define _FE_LOWLEVELENGINE_H

#include "../Utils/FELibDefines.h"

namespace FE
{
	/// Interface to low level image processing engines.
	class FELowLevelEngine
	{
	public:
		virtual void CopyImage(UChar4Image *image_out, const UChar4Image *image_in) const = 0;
		virtual void CopyImage(FloatImage *image_out, const FloatImage *image_in) const = 0;
		virtual void CopyImage(Float4Image *image_out, const Float4Image *image_in) const = 0;

		virtual void FilterSubsample(UChar4Image *image_out, const UChar4Image *image_in) const = 0;
		virtual void FilterSubsampleWithHoles(FloatImage *image_out, const FloatImage *image_in) const = 0;
		virtual void FilterSubsampleWithHoles(Float4Image *image_out, const Float4Image *image_in) const = 0;

		virtual void GradientX(Short4Image *grad_out, const UChar4Image *image_in) const = 0;
		virtual void GradientY(Short4Image *grad_out, const UChar4Image *image_in) const = 0;

		FELowLevelEngine(void) { }
		virtual ~FELowLevelEngine(void) { }
	};
}

#endif //_FE_LOWLEVELENGINE_H
