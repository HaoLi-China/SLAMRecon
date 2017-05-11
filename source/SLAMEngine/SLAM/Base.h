// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.

/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Ra¨²l Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/
 
#ifndef _BASE_H_
#define _BASE_H_

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
namespace SLAMRecon{
	 
#define FRAME_GRID_ROWS 48 
#define FRAME_GRID_COLS 64

	// Keypoints are assigned to cells in a grid. Grid inverse width and height.
	struct GridInfo {
		float m_fGridElementWidthInv;
		float m_fGridElementHeightInv;
	};
	
	// Calibration matrix and OpenCV distortion parameters.
	struct CameraInfo{
		// Calibration matrix
		cv::Mat m_K; 
		// OpenCV distortion parameters
		cv::Mat m_DistCoef;  
		 
		float m_fx;
		float m_fy;
		float m_cx;
		float m_cy;
		float m_invfx;
		float m_invfy;
		
		// Undistorted Image Bounds (computed once).
		float m_nMinX;
		float m_nMaxX;
		float m_nMinY;
		float m_nMaxY;
		
		// Stereo baseline multiplied by fx.
		float m_bf;
	};
	
	// Scale pyramid info.
	struct PyramidLevelInfo {
		int m_nScaleLevels; 
		float m_fScaleFactor; 
		float m_fLogScaleFactor;  
		std::vector<float> m_vScaleFactors; 
		std::vector<float> m_vInvScaleFactors;   
		std::vector<float> m_vLevelSigma2;  
		std::vector<float> m_vInvLevelSigma2;  
	};
} // namespace SLAMRecon


#endif // BASE_H