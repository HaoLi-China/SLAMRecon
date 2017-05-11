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

#ifndef _FRAME_H_
#define _FRAME_H_

#include <mutex>
#include <vector>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "Base.h"
#include "../ORB/ORBextractor.h"
#include "../ORB/ORBVocabulary.h"

namespace SLAMRecon {

	class KeyFrame;
	class MapPoint;
	  
	class Frame {

	public:

		Frame();
		
		// Copy constructor.
		Frame(const Frame &frame);
		
		// Constructor for RGB-D cameras.
		Frame(const cv::Mat &imGray, const cv::Mat &imDepth, ORBextractor* extractor, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, float m_bf, float m_fThDepth);
		
		~Frame();
		 
		void setRGBImg(cv::Mat rgbImg);
		
		// Extract ORB on the image.
		void ExtractORB(const cv::Mat &im);
		 
		void ComputeDepthFromRGBD(const cv::Mat &imDepth);
		
		// Set the camera pose.
		virtual void SetPose(cv::Mat Transformation);
		
		// Computes rotation, translation and camera center matrices from the camera pose.
		void UpdatePoseMatrices(); 
		
		// Returns the camera center.
		virtual cv::Mat GetCameraCenter();
		// Returns the camera rotation.
		virtual cv::Mat GetRotation();
		// Returns the camera translation.
		virtual cv::Mat GetTranslation();
		
		// Compute Bag of Words representation.
		void ComputeBoW();
		 
		cv::Mat ComputeWorldPos(const int &i);
		
		// Check if a MapPoint is in the frustum of the camera
		// and fill variables of the MapPoint to be used by the tracking
		bool isInFrustum(MapPoint* pMP, float viewingCosLimit);
		
		// Returns the feature indexes in area. (x,y) is the center coordinate and r is the radius of this area.
		vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel = -1, const int maxLevel = -1) const;
		
		//Check (x,y) is in the image.
		bool IsInImage(const float &x, const float &y) const;

	public:
		// Current and Next Frame id.
		static long unsigned int m_nFNextId;
		long unsigned int m_nFId;

		// RGB image
		cv::Mat m_rgbImg;
		 
		// Reference Keyframe of this frame.
		KeyFrame* m_pReferenceKF;
		
		// Vocabulary used for relocalization.
		ORBVocabulary* m_pORBvocabulary;
		
		// Feature extractor. The right is used only in the stereo case.
		ORBextractor* m_ORBextractor;
		
		// Number of KeyPoints.
		int m_nKeys;
		
		// Vector of keypoints (original for visualization) and undistorted (actually used by the system).
		// In the stereo case, mvKeysUn is redundant as images must be rectified.
		// In the RGB-D case, RGB images can be distorted.
		vector<cv::KeyPoint> m_vKeys;
		vector<cv::KeyPoint> m_vKeysUn;
		
		// MapPoints associated to keypoints, NULL pointer if no association.
		vector<MapPoint*> m_vpMapPoints;
		
		// Flag to identify outlier associations.
		vector<bool> m_vbOutlier;
		
		// ORB descriptor, each row associated to a keypoint.
		cv::Mat m_Descriptors;
		
		// Bag of Words Vector structures.
		DBoW2::BowVector m_BowVec; 
		DBoW2::FeatureVector m_FeatVec;  
		
		// Corresponding depth for each keypoint.
		std::vector<float> m_vuRight;
		std::vector<float> m_vfDepth;
		float m_fThDepth;
		
		// Camera pose.
		cv::Mat m_Transformation;  
		
		// Scale pyramid info.
		static PyramidLevelInfo* m_pPLevelInfo;
		
		// Calibration matrix and OpenCV distortion parameters.
		static CameraInfo* m_pCameraInfo;
		
		// Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
		static GridInfo* m_pGridInfo;
		std::vector<size_t> m_Grid[FRAME_GRID_COLS][FRAME_GRID_ROWS]; 
		
		// Only initialize once for pyramid info, camera calibration matrix, and so on.
		static bool m_bInitialComputations;
	
	private:
		
		// Undistort keypoints given OpenCV distortion parameters.
		// Only for the RGB-D case. Stereo must be already rectified!
		// (called in the constructor).
		void UndistortKeyPoints();
		
		// Computes image bounds for the undistorted image (called in the constructor).
		void ComputeImageBounds(const cv::Mat &im);
		
		// Assign keypoints to the grid for speed up feature matching (called in the constructor).
		void AssignFeaturesToGrid();
		 
		bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);
	
	private: 
		// Rotation, translation and camera center
		cv::Mat m_R;  
		cv::Mat m_t; 
		cv::Mat m_C;  
	};
}// namespace SLAMRecon

#endif // FRAME_H