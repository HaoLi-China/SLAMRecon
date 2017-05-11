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

#ifndef _MAP_POINT_H_
#define _MAP_POINT_H_

#include <memory>
#include <opencv2/core/core.hpp>
#include "Frame.h"

namespace SLAMRecon {
	class Map;
	class Frame;
	 
	class MapPoint {

	public: 
		MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
		 
		MapPoint(const cv::Mat &Pos, Frame* pFrame, const int &idxF, Map* pMap);

		~MapPoint();
		 
		void SetWorldPos(const cv::Mat &Pos);
		cv::Mat GetWorldPos();
		 
		cv::Mat GetNormal();
		float GetMinDistanceInvariance();
		float GetMaxDistanceInvariance();
		cv::Mat GetDescriptor();
		KeyFrame* GetReferenceKeyFrame();
		std::map<KeyFrame*, size_t> GetObservations();
		int Observations();
		 
		void IncreaseVisible(int n = 1);
		void IncreaseFound(int n = 1);
		inline int GetVisible(){
			return m_nVisible;
		}
		inline int GetFound(){
			return m_nFound;
		}  
		float GetFoundRatio(); 
		 
		int GetIndexInKeyFrame(KeyFrame* pKF);
		 
		bool IsInKeyFrame(KeyFrame* pKF);
		 
		void Replace(MapPoint* pMP); 
		MapPoint* GetReplaced();
		 
		void AddObservation(KeyFrame* pKF, size_t idx);
		 
		void EraseObservation(KeyFrame* pKF);
		 
		void SetBadFlag();
		bool isBad();
		 
		void ComputeDistinctiveDescriptors();
		 
		void UpdateNormalAndDepth();
		 
		int PredictScale(const float &currentDist, const float &logScaleFactor);

	public:
		 
		static long unsigned int n_MPNextId;
		 
		long unsigned int m_nMPId;
		 
		long unsigned int m_nLastFrameSeen;
		 
		long unsigned int m_nTrackReferenceForFrame;
		
		// Variables used by the tracking
		bool m_bTrackInView;  
		float m_TrackProjX;  
		float m_TrackProjY;
		int m_nTrackScaleLevel;  
		float m_TrackViewCos;  

		long unsigned int m_nFirstKFid;
		 
		// Variables used by local mapping
		long unsigned int m_nFuseCandidateForKF;
		long unsigned int m_nBALocalForKF;
		 
		// Variables used by loop closing
		long unsigned int m_nCorrectedByKF; 
		long unsigned int m_nCorrectedReference;
		long unsigned int m_nLoopPointForKF;
		cv::Mat m_PosGBA;
		long unsigned int m_nBAGlobalForKF;  

		static std::mutex m_GlobalMutex;

	private: 
		// Position in absolute coordinates
		cv::Mat m_WorldPos;
		
		// Keyframes observing the point and associated index in keyframe
		std::map<KeyFrame*, size_t> m_Observations;
		 
		int m_nObs;
		 
		// Mean viewing direction
		cv::Mat m_NormalVector; 

		// Best descriptor to fast matching
		cv::Mat m_Descriptor;

		// Scale invariance distances
		float m_fMinDistance;
		float m_fMaxDistance;
		 
		// Reference KeyFrame
		KeyFrame* m_pRefKF;

		// Tracking counters
		int m_nVisible;  
		int m_nFound; 
		 
		// Bad flag (we do not currently erase MapPoint from memory)
		bool m_bBad;
		MapPoint* m_pReplaced;
		 
		Map* m_pMap;
		 
		std::mutex m_MutexPos;  
		std::mutex m_MutexObservations;
	};
} // namespace SLAMRecon

#endif // MAPPOINT_H