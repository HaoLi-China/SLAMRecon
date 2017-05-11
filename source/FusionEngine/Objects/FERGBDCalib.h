// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM
#ifndef _FE_RGBDCALIB_H
#define _FE_RGBDCALIB_H

#include "FEIntrinsics.h"
#include "FEExtrinsics.h"
#include "FEDisparityCalib.h"

namespace FE
{
	/** \brief
		Represents the joint RGBD calibration parameters
		*/
	class FERGBDCalib
	{
	public:
		/// Intrinsic parameters of the RGB camera.
		FEIntrinsics intrinsics_rgb;

		/// Intrinsic parameters of the depth camera.
		FEIntrinsics intrinsics_d;

		/** @brief
			Extrinsic calibration between RGB and depth
			cameras.

			This transformation takes points from the RGB
			camera coordinate system to the depth camera
			coordinate system.
			*/
		FEExtrinsics trafo_rgb_to_depth;

		/// Calibration information to compute depth from disparity images.
		FEDisparityCalib disparityCalib;
	};
}
#endif //_FE_RGBDCALIB_H