/** 
*Copyright 2016 - 2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
*Copyright 2014 - 2015 Isis Innovation Limited and the authors of InfiniTAM.
*/

#include "Calibration.h"
using namespace Basis;
//Intrinsics intrinsics_d;
//intrinsics_d.setFrom(580, 580, 320, 240, depthImageSize.x, depthImageSize.y);//for kinect1 or xtion
//intrinsics_d.setFrom(366.685, 366.685, 256.52, 208.1, depthImageSize.x, depthImageSize.y);//for kinect2
//Intrinsics intrinsics_rgb;
//intrinsics_rgb.setFrom(0, 0, 0, 0, rgbImageSize.x, rgbImageSize.y);
//Calibration calib = new Calibration(intrinsics_rgb, intrinsics_d);

Calibration::Calibration(const Intrinsics &intrinsics_rgb, const Intrinsics &intrinsics_d){
	this->intrinsics_rgb = intrinsics_rgb;
	this->intrinsics_d = intrinsics_d;
}

Calibration::~Calibration(){

}
void Intrinsics::setFrom(double fx, double fy, double cx, double cy, double width, double height)
{
	this->fx = fx;
	this->fy = fy;
	this->px = cx;
	this->py = cy;
	this->all.x() = fx;
	this->all.y() = fy;
	this->all.z() = cx;
	this->all.w() = cy;
	this->imageSize.x() = width;
	this->imageSize.y() = height;
}

void Calibration::depthImgToPoints(const EVector4d &projParams, const int width, const int height, const std::vector<double> &depth, std::vector<EVector3> &points){
	points.clear();

	for (int y = 0; y < height; y++){
		for (int x = 0; x < width; x++){
			int id = width * y + x;

			if (depth[id] > 0){
				EVector3d pt;
				pt.z() = depth[id];
				pt.x() = pt.z() * ((float(x) - projParams.z()) * 1.0 / projParams.x());
				pt.y() = pt.z() * ((float(y) - projParams.w()) * 1.0 / projParams.y());

				points.push_back(pt);
			}
		}
	}
}

void Calibration::pointToPixPos(const EVector4d &projParams, const int width, const int height, const EVector3 &point, EVector2d &pixPos){
	if (point.z() <= 0) {
		pixPos.x() = -1;
		pixPos.y() = -1;
	}

	pixPos.x() = projParams.x() * point.x() / point.z() + projParams.z();
	pixPos.y() = projParams.y() * point.y() / point.z() + projParams.w();
}