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

#ifndef _SLAM_H_
#define _SLAM_H_

#include <string>
#include <thread>
#include <opencv2/core/core.hpp>

#include "Map.h"
#include "KeyFrameDatabase.h"
#include "../ORB/ORBVocabulary.h"
#include "../ORB/ORBextractor.h"
#include "../ORB/ORBmatcher.h"
#include "Tracking.h"
#include "LocalMapping.h"

namespace SLAMRecon
{

	class SLAM
	{
	public:
		// Initialize slam system.
		SLAM(const string &strVocFile, const string &strSettingFile, Map* pMap, CovisibilityGraph* cograph, SpanningTree* spantree);
		~SLAM();

		void Shutdown(bool &ShutdowmFlag);

		void Reset();

		// tracking process
		cv::Mat trackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp);

		Map* getMap();

	protected:

	private:
		// Map structure that stores the pointers to all KeyFrames and MapPoints.
		Map* m_pMap;
		ORBVocabulary* m_pORBVocabulary;
		

		KeyFrameDatabase* m_pKeyFrameDatabase;

		CovisibilityGraph* m_pCoGraph;
		SpanningTree* m_pSpanTree;

		Tracking* m_pTracker;
		LocalMapping* m_pLocalMapper;
		LoopClosing* m_pLoopCloser;

		std::thread* m_ptLocalMapping;
		std::thread* m_ptLoopClosing;
	};
} // namespace SLAMRecon

#endif // SLAM_H