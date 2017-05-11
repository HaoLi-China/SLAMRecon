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

#include "SLAM.h"
using namespace std;

namespace SLAMRecon {

	SLAM::SLAM(const string &strVocFile, const string &strSettingsFile, Map* pMap, CovisibilityGraph* cograph, SpanningTree* spantree) {
		//Check settings file
		cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
		if (!fsSettings.isOpened())
		{
			cerr << "Failed to open settings file at: " << strSettingsFile << endl;
			exit(-1);
		}

		//Load ORB Vocabulary
		cout << "Loading ORB Vocabulary. This could take a while..." << endl;

		m_pORBVocabulary = new ORBVocabulary();
		bool bVocLoad = m_pORBVocabulary->loadFromTextFile(strVocFile);
		// bool bVocLoad = true;
		if (!bVocLoad)
		{
			cerr << "Wrong path to vocabulary. " << endl;
			cerr << "Falied to open at: " << strVocFile << endl;
			exit(-1);
		}
		cout << "Vocabulary loaded!" << endl;

		//
		/*m_pCoGraph = new CovisibilityGraph();
		m_pSpanTree = new SpanningTree(m_pCoGraph);*/
		m_pCoGraph = cograph;
		m_pSpanTree = spantree;

		// KeyFrame Database
		m_pKeyFrameDatabase = new KeyFrameDatabase(m_pORBVocabulary, m_pCoGraph);

		//
		// m_pMap = new Map();
		m_pMap = pMap;

		//
		m_pTracker = new Tracking(strSettingsFile, m_pORBVocabulary, m_pCoGraph, m_pSpanTree, m_pKeyFrameDatabase, m_pMap);

		m_pLocalMapper = new LocalMapping(m_pMap, m_pKeyFrameDatabase, m_pCoGraph, m_pSpanTree);

		m_pLoopCloser = new LoopClosing(m_pMap, m_pKeyFrameDatabase, m_pORBVocabulary, m_pCoGraph, m_pSpanTree);


		m_pTracker->SetLocalMapper(m_pLocalMapper);
		m_pTracker->SetLoopCloser(m_pLoopCloser);

		m_pLocalMapper->SetTracker(m_pTracker);
		m_pLocalMapper->SetLoopCloser(m_pLoopCloser);

		m_pLoopCloser->SetTracker(m_pTracker);
		m_pLoopCloser->SetLocalMapper(m_pLocalMapper);

		m_ptLocalMapping = new thread(&LocalMapping::Run, m_pLocalMapper);
		m_ptLoopClosing = new thread(&LoopClosing::Run, m_pLoopCloser);
	}

	SLAM::~SLAM() {
		
		// stop threads
		if (!m_pLocalMapper->isFinished() || !m_pLoopCloser->isFinished()) {
			bool flag;
			Shutdown(flag);
		}

		// delete instance object
		Frame::m_nFNextId = 0;
		Frame::m_bInitialComputations = true;

		if (Frame::m_pPLevelInfo != NULL)
			delete Frame::m_pPLevelInfo;
		if (Frame::m_pCameraInfo != NULL)
			delete Frame::m_pCameraInfo;
		if (Frame::m_pGridInfo != NULL)
			delete Frame::m_pGridInfo;

		KeyFrame::m_nKFNextId = 0;

		// delete runner
		if (m_pTracker != NULL) 
			delete m_pTracker;
		
		if (m_pLocalMapper != NULL) 
			delete m_pLocalMapper;
		
		if (m_pLoopCloser != NULL) 
			delete m_pLoopCloser;
		
		// delete threads
		m_ptLocalMapping->join();
		m_ptLoopClosing->join();
		if (m_ptLocalMapping != NULL)
			delete m_ptLocalMapping;
		if (m_ptLoopClosing != NULL)
			delete m_ptLoopClosing;
		
		// delete attributes
		if (m_pORBVocabulary != NULL) {
			delete m_pORBVocabulary;
		}
		if (m_pKeyFrameDatabase != NULL) {
			delete m_pKeyFrameDatabase;
		}

		
	}

	void SLAM::Shutdown(bool &ShutdowmFlag) {

		m_pLocalMapper->RequestFinish();
		m_pLoopCloser->RequestFinish();

		// Wait until all thread have effectively stopped
		while (!m_pLocalMapper->isFinished() || !m_pLoopCloser->isFinished() || m_pLoopCloser->isRunningGBA()) {
			Sleep(5);
		}

		cout << "The Slam is shutdown!!!" << endl;

		ShutdowmFlag = true;
	}

	void SLAM::Reset() {
		m_pTracker->Reset();
	}

	cv::Mat SLAM::trackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp) {
		return m_pTracker->GrabImageRGBD(im, depthmap);
	}

	Map* SLAM::getMap() {
		return m_pMap;
	}

} // namespace SLAMRecon