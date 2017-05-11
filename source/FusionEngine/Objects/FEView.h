// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM
#ifndef _FE_VIEW_H
#define _FE_VIEW_H

#include "../Objects/FERGBDCalib.h"
#include "Define.h"

namespace FE
{
		/** 
		    Represents a single "view", i.e. RGB and depth images along
		    with all intrinsic and relative calibration information
		*/
		class FEView
		{
		public:
			/// Intrinsic calibration information for the view.
			FERGBDCalib *calib;

			/// RGB colour image.
			UChar4Image *rgb; 

			/// Float valued depth image, if available according to @ref inputImageType.
			FloatImage *depth;

			/// surface normal of depth image
			// allocated when needed
			Float4Image *depthNormal;

			/// uncertainty (std) in each pixel of depth value based on sensor noise model
			/// allocated when needed
			FloatImage *depthUncertainty;

			FEView(const FERGBDCalib *calibration, Vector2i imgSize_rgb, Vector2i imgSize_d, bool useGPU)
			{
				this->calib = new FERGBDCalib(*calibration);
				this->rgb = new UChar4Image(imgSize_rgb, true, useGPU);
				this->depth = new FloatImage(imgSize_d, true, useGPU);
				this->depthNormal = NULL;
				this->depthUncertainty = NULL;
			}

			virtual ~FEView(void)
			{
				delete calib;

				delete rgb;
				delete depth;

				if (depthNormal != NULL) delete depthNormal;
				if (depthUncertainty != NULL) delete depthUncertainty;
			}

			// Suppress the default copy constructor and assignment operator
			FEView(const FEView&);
			FEView& operator=(const FEView&);
		};
}

#endif //_FE_VIEW_H