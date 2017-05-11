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

#ifndef _MAP_H_
#define _MAP_H_

#include <vector>
#include <memory>
#include <mutex>
#include <set>
#include "MapPoint.h"
#include "KeyFrame.h"
#include "SpanningTree.h"

namespace SLAMRecon {
	class MapPoint;
	class KeyFrame;
	class Frame; 

	class Map {

	public:
		 
		Map();
		~Map();
		
		// Add/Erase KeyFrame/MapPoint.
		void AddKeyFrame(KeyFrame* pKF);
		void AddMapPoint(MapPoint* pMP);
		void EraseMapPoint(MapPoint* pMP);
		void EraseKeyFrame(KeyFrame* pKF);
		 
		// Returns of all KeyFrames/MapPoints.
		std::vector<KeyFrame*> GetAllKeyFrames();
		std::vector<MapPoint*> GetAllMapPoints();

		// Frame, only used for debug.
		void AddFrame(Frame* pFrame);
		std::vector<Frame*> GetAllFrames();
		 
		// Returns the number of KeyFrames/MapPoints in the map.
		long unsigned int MapPointsInMap();
		long unsigned int KeyFramesInMap();
		
		std::vector<MapPoint*> GetReferenceMapPoints();
		void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
		 
		void clear();
		 
		long unsigned int GetMaxKFid();

	// Below is used for final result saved in the map.
	public:
		 
		std::vector<KeyFrame*> m_vpKeyFrameOrigins;
		 
		std::mutex m_MutexMapUpdate;
		std::list<cv::Mat> m_lRelativeFramePoses;
		std::list<KeyFrame*> m_lpReferences;  
		std::list<bool> m_lbLost;  
		std::list<bool> m_lbKF;
	private:
		std::mutex m_MutexRelativeInfo;
		 
		std::list<std::pair<int, cv::Mat> > m_lpFIdAndPoses;
		std::mutex m_MutexFIdAndPoses;
		 
		std::list<KeyFrame*> m_modifiedKeyFrames;
		std::mutex m_MutexModifiedKeyFrames;


	public:
		void addRelativeInfo(cv::Mat RelativeFramePose, KeyFrame* pReferenceKF, bool bLost, bool bKF);
		void addLastRelativeInfo(bool bLost); 
		std::list<std::pair<int, cv::Mat> > getFramesByKF(KeyFrame* pKF, SpanningTree *pSpanTree);
		  
		void addIdAndPose(int fId, cv::Mat pose);
		std::pair<int, cv::Mat>  Map::getIdAndPose();

		void addModifiedKeyFrame(KeyFrame* pKF);
		KeyFrame* getModifiedKeyFrame();
		 
		cv::Mat m_CurFramePose;
		void setCurFramePose(cv::Mat curFramePose);
		cv::Mat getCurFramePose();
		std::mutex m_MutexCurFramePose;
	// Above is used for final result saved in the map.

	private: 
		std::set<MapPoint*> m_spMapPoints;
		 
		std::set<KeyFrame*> m_spKeyFrames;
		 
		std::set<Frame*> m_spFrames;
		 
		std::vector<MapPoint*> m_vpReferenceMapPoints;
		 
		long unsigned int m_nMaxKFid;
		 
		std::mutex m_MutexMap;
		 
	};
} // namespace SLAMRecon

#endif // MAP_H