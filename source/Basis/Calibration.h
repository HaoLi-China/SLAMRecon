#ifndef _CALIBRATION_H
#define _CALIBRATION_H
/**
* This file defines some structures and functions about camera projection.
*
* Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
* Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM.
*/

#include "EigenDefine.h"

#include <vector>

namespace Basis
{
	/** The actual intrinsic calibration parameters.*/
	struct Intrinsics
	{
		EVector4d all;
		double fx, fy, px, py;
		EVector2i imageSize;

		void setFrom(double fx, double fy, double cx, double cy, double width, double height);
	};

	class Calibration{
	public:
		Calibration(const Intrinsics &intrinsics_rgb, const Intrinsics &intrinsics_d);
		~Calibration();

		Intrinsics intrinsics_rgb;
		Intrinsics intrinsics_d;

		static void depthImgToPoints(const EVector4d &projParams, const int width, const int height, const std::vector<double> &depth, std::vector<EVector3> &points);
		static void pointToPixPos(const EVector4d &projParams, const int width, const int height, const EVector3 &point, EVector2d &pixPos);
	};
}

#endif