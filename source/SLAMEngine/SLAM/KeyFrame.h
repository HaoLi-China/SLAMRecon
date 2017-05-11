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

#ifndef _KEY_FRAME_H_
#define _KEY_FRAME_H_

#include "Frame.h"

namespace SLAMRecon {

	class Frame;
	class MapPoint;
	class CovisibilityGraph;
	class SpanningTree;
	class Map;
	class KeyFrameDatabase;

	class KeyFrame : public Frame {
	
	public:
	
		KeyFrame();
		KeyFrame(Frame& frame);
		~KeyFrame();
		 
		// Set and get the camera pose.
		void SetPose(cv::Mat Transformation);
		cv::Mat GetPose();
		cv::Mat GetPoseInverse();
		cv::Mat GetCameraCenter();
		cv::Mat GetRotation();
		cv::Mat GetTranslation();
		 
		// MapPoint observation functions
		void AddMapPoint(MapPoint* pMP, const size_t &idx);
		MapPoint* GetMapPoint(const size_t &idx);
		void EraseMapPoint(const size_t &idx);
		void EraseMapPoint(MapPoint* pMP);
		void ReplaceMapPoint(const size_t &idx, MapPoint* pMP);
		std::set<MapPoint* > GetMapPoints();
		std::vector<MapPoint*> GetMapPointMatches();
		int TrackedMapPoints(const int &minObs);
		
		// Set/check bad flag
		void SetBadFlag(CovisibilityGraph* pCograph, SpanningTree* pSpanTree, KeyFrameDatabase* pKFDB, Map* pMap);
		void SetBad(KeyFrame *pPKF);
		bool isBad();
		
		// Enable/Disable bad flag changes
		void SetNotErase();
		void SetErase(CovisibilityGraph* pCograph, SpanningTree* pSpanTree, KeyFrameDatabase* pKFDB, Map* pMap);

		static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
			return pKF1->m_nKFId < pKF2->m_nKFId;
		}
	public: 
		static long unsigned int m_nKFNextId;
		 
		long unsigned int m_nKFId;
		
		// Variables used by the keyframe database
		long unsigned int mnRelocQuery; 
		int mnRelocWords; 
		float mRelocScore; 
		long unsigned int m_nLoopQuery;  
		int m_nLoopWords; 
		float m_LoopScore; 
		
		// Variables used by the tracking
		long unsigned int m_nTrackReferenceForFrame; 
		long unsigned int m_nFuseTargetForKF;     
		
		// Variables used by the local mapping
		long unsigned int m_nBALocalForKF;
		long unsigned int m_nBAFixedForKF;
		
		// Variables used by loop closing
		cv::Mat m_TcwGBA;
		cv::Mat m_TcwBefGBA;  
		long unsigned int m_nBAGlobalForKF;  
		  
		cv::Mat m_Tcp;
		 
		std::mutex m_MutexMapPoints; 
		std::mutex m_MutexConnections;
		 
		bool m_FirstFusion;
		cv::Mat m_oldCameraPose;
		 
	protected:
		 
		bool m_bBad;
		bool m_bNotErase;
		bool m_bToBeErased;
		 
		std::mutex m_MutexPose;
		std::mutex m_MutexBad;
	};
	
} // namespace SLAMRecon

#endif // KEYFRAME_H