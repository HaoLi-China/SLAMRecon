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

#ifndef _LOCAL_MAPPING_H
#define _LOCAL_MAPPING_H

#include "LoopClosing.h"
#include "Tracking.h"
#include "CovisibilityGraph.h"
#include "SpanningTree.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include <mutex>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

namespace SLAMRecon {

	class LoopClosing;
	class Tracking;

	class LocalMapping {
	public:
		LocalMapping(Map* pMap, KeyFrameDatabase* pDB, CovisibilityGraph* pCoGraph, SpanningTree* pSpanTree);
		~LocalMapping();

		void SetLoopCloser(LoopClosing* pLoopCloser);
		void SetTracker(Tracking* pTracker);

		// main thread
		void Run();


		void InsertKeyFrame(KeyFrame* pKF);

		// Thread Synch
		void RequestStop();
		
		bool Stop();
		void Release();
		bool isStopped();
		bool stopRequested();

		bool AcceptKeyFrames();
		void SetAcceptKeyFrames(bool flag);
		bool SetNotStop(bool flag);


		void RequestFinish();
		bool isFinished();

		// 


		void InterruptBA();

		int KeyframesInQueue(){
			unique_lock<mutex> lock(m_MutexNewKFs);
			return m_lpNewKeyFrames.size();
		}

		// Tracking request reset
		void RequestReset();

	protected:

		bool CheckNewKeyFrames();
		
		// KeyFrame Insertion
		void ProcessNewKeyFrame();

		// Recent Map Points Culling
		void MapPointCulling();

		// New Map Point Creation
		void CreateNewMapPoints();
		cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);  
		cv::Mat SkewSymmetricMatrix(const cv::Mat &v);  


		void SearchInNeighbors();

		void KeyFrameCulling();

		bool CheckFinish();
		void SetFinish();
		

		

	protected:
		Map* m_pMap;

		LoopClosing* m_pLoopCloser;
		Tracking* m_pTracker;

		list<KeyFrame*> m_lpNewKeyFrames;
		KeyFrame* m_pCurrentKeyFrame;

		list<MapPoint*> m_lpRecentAddedMapPoints;

		CovisibilityGraph* m_pCoGraph;
		SpanningTree* m_pSpanTree;

		KeyFrameDatabase* m_pKeyFrameDB;

		bool m_bFinishRequested;
		bool m_bFinished;
		mutex m_MutexFinish;

		 
		void ResetIfRequested();
		bool m_bResetRequested;
		std::mutex m_MutexReset;


		mutex m_MutexNewKFs;

		bool m_bAbortBA;

		bool m_bStopped;
		bool m_bStopRequested;
		bool m_bNotStop;
		mutex m_MutexStop;

		bool m_bAcceptKeyFrames;
		mutex m_MutexAccept;
	};
} // namespace SLAMRecon

#endif // LOCALMAPPING_H
