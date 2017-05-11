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

#ifndef _LOOP_CLOSING_H
#define _LOOP_CLOSING_H

#include "Map.h"
#include "Tracking.h"
#include "LocalMapping.h"
#include "KeyFrameDatabase.h"
#include "../ORB/ORBVocabulary.h"
#include <g2o/types/sim3/sim3.h>

#include <thread>
#include <mutex>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

namespace SLAMRecon {

	class Tracking;
	class LocalMapping;

	class LoopClosing {
	
	public:
		 
		typedef pair<set<KeyFrame*>, int> ConsistentGroup;
		typedef map<KeyFrame*, g2o::Sim3, std::less<KeyFrame*>,
			Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;

	public:
		LoopClosing(Map* pMap, KeyFrameDatabase* pDB, ORBVocabulary* pVoc, CovisibilityGraph* pCoGraph, SpanningTree* pSpanTree);
		~LoopClosing();
		 
		void SetTracker(Tracking* pTracker);
		void SetLocalMapper(LocalMapping* pLocalMapper);

		// main thread
		void Run();
		 
		void InsertKeyFrame(KeyFrame *pKF);
		 
		void RequestFinish();
		bool isFinished();
		bool isRunningGBA(){
			unique_lock<std::mutex> lock(m_MutexGBA);
			return m_bRunningGBA;
		}
		bool isFinishedGBA(){
			unique_lock<std::mutex> lock(m_MutexGBA);
			return m_bFinishedGBA;
		}

		// tracking request reset
		void RequestReset();

	protected: 
		bool CheckNewKeyFrames();
		 
		bool DetectLoop();
		 
		bool ComputeSim3();
		 
		void CorrectLoop();
		 
		void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap);
		 
		// This function will run in a separate thread
		void RunGlobalBundleAdjustment(unsigned long nLoopKF);
		 
	protected:
		 
		bool CheckFinish();
		void SetFinish();
		
		bool m_bFinishRequested;
		bool m_bFinished;
		std::mutex m_MutexFinish;
		 
		std::list<KeyFrame*> m_lpLoopKeyFrameQueue;
		std::mutex m_MutexLoopQueue;
		
		// Loop detector variables
		KeyFrame* m_pCurrentKF;
		 
		std::vector<ConsistentGroup> m_vConsistentGroups;
		 
		std::vector<KeyFrame*> m_vpCurrentConnectedKFs;
		 
		KeyFrame* m_pMatchedKF;
		 
		std::vector<MapPoint*> m_vpCurrentMatchedPoints;
		 
		std::vector<MapPoint*> m_vpLoopMapPoints;
		 
		cv::Mat m_Scw;
		g2o::Sim3 m_g2oScw;
		 
		long unsigned int m_LastLoopKFid;
		 
		float m_nCovisibilityConsistencyTh;

		 
		std::vector<KeyFrame*> m_vpEnoughConsistentCandidates;
		 
		Map* m_pMap;
		 
		Tracking* m_pTracker;
		LocalMapping *m_pLocalMapper;
		
		KeyFrameDatabase* m_pKeyFrameDB;
		 
		CovisibilityGraph* m_pCoGraph;
		SpanningTree* m_pSpanTree;
		 
		ORBVocabulary* m_pORBVocabulary;

		// Global Bundle Adjustment
		bool m_bRunningGBA;
		bool m_bFinishedGBA;
		bool m_bStopGBA;
		std::mutex m_MutexGBA; 
		std::thread* m_pThreadGBA;

		//
		void ResetIfRequested();
		bool m_bResetRequested;
		std::mutex m_MutexReset;
	};

} // namespace SLAMRecon

#endif // LOOPCLOSING_H