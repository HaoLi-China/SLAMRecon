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

#ifndef _TRACKING_H_
#define _TRACKING_H_

#include <opencv2/core/core.hpp>
#include <vector>
#include <list>
#include <string>
#include <iostream>
#include "Frame.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "../ORB/ORBextractor.h"
#include "../ORB/ORBmatcher.h"
#include "CovisibilityGraph.h"
#include "SpanningTree.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "PnPsolver.h"

namespace SLAMRecon
{
	class LocalMapping;
	class LoopClosing;

	class Tracking
	{
	public:
		Tracking(const string &strSettingPath, ORBVocabulary* voc, CovisibilityGraph* cograph, SpanningTree* spantree, KeyFrameDatabase* keyFrameDatabase, Map *pMap);
		~Tracking();

		// Set another threads object.
		void SetLocalMapper(LocalMapping* pLocalMapper);
		void SetLoopCloser(LoopClosing* pLoopCloser);

		// Main function.
		cv::Mat GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD);
		// 
		void Reset();

	public:
		// Tracking states
		enum eTrackingState{
			SYSTEM_NOT_READY = -1,
			NO_IMAGES_YET = 0,
			NOT_INITIALIZED = 1,
			OK = 2,
			LOST = 3
		};
		eTrackingState m_State;

		// Current Frame
		Frame m_CurrentFrame;
		cv::Mat m_ImageGray;
		
		// Lists used to recover the full camera trajectory at the end of the execution.
		// Basically we store the reference keyframe for each frame and its relative transformation
		// Not used, move to the map file.
		list<cv::Mat> m_lRelativeFramePoses;
		list<KeyFrame*> m_lpReferences;
		list<bool> m_lbLost; 
		list<bool> m_bKF;

	protected:

		// Main tracking function. It is independent of the input sensor.
		void Track();

		// Map initialization.
		void Initialization();

		// Tracking based on KeyFrame.
		bool TrackReferenceKeyFrame();

		// Tracking based on motion model.
		bool TrackWithMotionModel();

		void UpdateLastFrame();

		// Relocalization
		bool Relocalization();

		// LocalMap
		bool TrackLocalMap();
		void UpdateLocalMap();
		void UpdateLocalKeyFrames();
		void UpdateLocalMapPoints();


		void SearchLocalPoints();

		// Check map is needed KeyFrame or not.
		bool NeedNewKeyFrame();

		// Create new KeyFrame.
		void CreateNewKeyFrame();


	protected:
		
		// Calibration matrix and OpenCV distortion parameters.
		cv::Mat m_K;
		cv::Mat m_DistCoef;
		// Stereo baseline multiplied by fx.
		float m_bf;

		//New KeyFrame rules (according to fps)
		int m_MinFrames;
		int m_MaxFrames;

		bool m_RGB;
		
		// Threshold close/far points
		// Points seen as close by the stereo/RGBD sensor are considered reliable
		// and inserted from just one frame. Far points requiere a match in two keyframes.
		float m_fThDepth;

		// For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
		float m_DepthMapFactor;

		//Current matches in frame
		int m_nMatchesInliers;

		//Last Frame, KeyFrame and Relocalisation Info
		Frame m_LastFrame;
		KeyFrame* m_pLastKeyFrame;
		unsigned int m_nLastKeyFrameId;
		unsigned int m_nLastRelocFrameId;

		// Temporary MapPoints for tracking
		list<MapPoint*> m_lpTemporalPoints;

		
		//Motion Model
		cv::Mat m_Velocity;

		// ORB
		ORBextractor* m_pORBextractor;
		ORBVocabulary* m_pORBVocabulary;

		// KeyFrame Database 
		KeyFrameDatabase* m_pKeyFrameDB;

		// CovisibilityGraph and SpanningTree
		CovisibilityGraph* m_pCoGraph;
		SpanningTree* m_pSpanTree;

		// Local Map
		KeyFrame* m_pReferenceKF; 
		vector<KeyFrame*> m_vpLocalKeyFrames;  
		vector<MapPoint*> m_vpLocalMapPoints;  

		// Map
		Map* m_pMap;

		// Other Thread Pointers
		LocalMapping* m_pLocalMapper;
		LoopClosing* m_pLoopCloser;

	};
} // namespace SLAMRecon

#endif // TRACKING_H