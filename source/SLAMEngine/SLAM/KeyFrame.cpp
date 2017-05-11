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

#include "KeyFrame.h"
#include "MapPoint.h"
#include "../ORB/ORBextractor.h"
#include "../ORB/ORBVocabulary.h"
#include "Converter.h"
#include "CovisibilityGraph.h"
#include "SpanningTree.h"
#include "Map.h"
#include "KeyFrameDatabase.h"


using namespace std;

namespace SLAMRecon {
	long unsigned int KeyFrame::m_nKFNextId = 0;

	KeyFrame::KeyFrame() :Frame() {}

	KeyFrame::KeyFrame(Frame& frame) 
		: Frame(frame), m_nTrackReferenceForFrame(0), m_nFuseTargetForKF(0), mnRelocQuery(0), mnRelocWords(0), mRelocScore(0),
		m_bNotErase(false), m_bToBeErased(false), m_bBad(false), m_FirstFusion(true)
	{
		m_nKFId = m_nKFNextId++;
		 
		m_pReferenceKF = static_cast<KeyFrame*>(NULL);
	}

	KeyFrame::~KeyFrame(){

	}

	void KeyFrame::SetPose(cv::Mat Transformation) {
		unique_lock<mutex> lockMPs(m_MutexPose);
		Transformation.copyTo(m_Transformation);
		cv::Mat R = m_Transformation.rowRange(0, 3).colRange(0, 3);
		cv::Mat t = m_Transformation.rowRange(0, 3).col(3);
		cv::Mat C = -R.t()*t;
	}

	cv::Mat KeyFrame::GetPose() {
		unique_lock<mutex> lockMPs(m_MutexPose);
		return m_Transformation.clone();
	}

	cv::Mat KeyFrame::GetPoseInverse() {
		unique_lock<mutex> lockMPs(m_MutexPose);
		cv::Mat R = m_Transformation.rowRange(0, 3).colRange(0, 3);
		cv::Mat t = m_Transformation.rowRange(0, 3).col(3);

		// m_R * x3Dw + m_t = x3Dc -> x3Dw = m_R.t() * x3Dc - m_R.t() * m_t 
		cv::Mat p_Transformation = cv::Mat::eye(4, 4, m_Transformation.type());
		cv::Mat p_R = R.t();
		p_R.copyTo(p_Transformation.rowRange(0, 3).colRange(0, 3));
		cv::Mat c_C = -p_R*t;
		c_C.copyTo(p_Transformation.rowRange(0, 3).col(3));
		return p_Transformation.clone();
	}

	cv::Mat KeyFrame::GetCameraCenter() {
		unique_lock<mutex> lockMPs(m_MutexPose);
		cv::Mat R = m_Transformation.rowRange(0, 3).colRange(0, 3);
		cv::Mat t = m_Transformation.rowRange(0, 3).col(3);
		return -R.t()*t.clone();
	}

	cv::Mat KeyFrame::GetRotation() {
		unique_lock<mutex> lockMPs(m_MutexPose);
		return m_Transformation.rowRange(0, 3).colRange(0, 3).clone();
	}

	cv::Mat KeyFrame::GetTranslation() {
		unique_lock<mutex> lockMPs(m_MutexPose);
		return m_Transformation.rowRange(0, 3).col(3).clone();
	}

	void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx) {
		unique_lock<mutex> lockMPs(m_MutexMapPoints);
		m_vpMapPoints[idx] = pMP;
	}

	MapPoint* KeyFrame::GetMapPoint(const size_t &idx) {
		unique_lock<mutex> lockMPs(m_MutexMapPoints);
		return m_vpMapPoints[idx];
	}

	void KeyFrame::EraseMapPoint(const size_t &idx) {
		unique_lock<mutex> lockMPs(m_MutexMapPoints);
		m_vpMapPoints[idx] = static_cast<MapPoint*>(NULL);
	}

	void KeyFrame::EraseMapPoint(MapPoint* pMP) {
		unique_lock<mutex> lockMPs(m_MutexMapPoints);
		int idx = pMP->GetIndexInKeyFrame(this);
		if (idx >= 0)
			m_vpMapPoints[idx] = static_cast<MapPoint*>(NULL);
	}

	void KeyFrame::ReplaceMapPoint(const size_t &idx, MapPoint* pMP) {
		unique_lock<mutex> lockMPs(m_MutexMapPoints);
		m_vpMapPoints[idx] = pMP;
	}

	set<MapPoint*> KeyFrame::GetMapPoints() {
		unique_lock<mutex> lockMPs(m_MutexMapPoints);
		set<MapPoint*> s;
		for (size_t i = 0, iend = m_vpMapPoints.size(); i < iend; i++) {
			if (!m_vpMapPoints[i])
				continue;
			MapPoint* pMP = m_vpMapPoints[i];
			if (!pMP->isBad())
				s.insert(pMP);
		}
		return s;
	}

	vector<MapPoint*> KeyFrame::GetMapPointMatches() {
		unique_lock<mutex> lockMPs(m_MutexMapPoints);
		return m_vpMapPoints;
	}

	int KeyFrame::TrackedMapPoints(const int &minObs) {
		unique_lock<mutex> lockMPs(m_MutexMapPoints);
		int nPoints = 0;
		const bool bCheckObs = minObs > 0;
		for (int i = 0; i < m_nKeys; i++) {
			MapPoint* pMP = m_vpMapPoints[i];
			if (pMP) {
				if (!pMP->isBad()) {
					if (bCheckObs) {
						if (m_vpMapPoints[i]->Observations() >= minObs)
							nPoints++;
					}
					else
						nPoints++;
				}
			}
		}
		return nPoints;
	}
	
	void KeyFrame::SetNotErase() {
		unique_lock<mutex> lock(m_MutexConnections);
		m_bNotErase = true;
	}

	void KeyFrame::SetErase(CovisibilityGraph* pCograph, SpanningTree* pSpanTree, KeyFrameDatabase* pKFDB, Map* pMap) {
		{
			unique_lock<mutex> lock(m_MutexConnections);
			if (pSpanTree->GetLoopEdges(this).empty()){
				m_bNotErase = false;  
			}
		}

		if (m_bToBeErased) {
			SetBadFlag(pCograph, pSpanTree, pKFDB, pMap);
		}
	}

	void KeyFrame::SetBadFlag(CovisibilityGraph* pCograph, SpanningTree* pSpanTree, KeyFrameDatabase* pKFDB, Map* pMap) {
		{
			unique_lock<mutex> lock(m_MutexConnections);
			if (m_nKFId == 0)
				return;
			else if (m_bNotErase) {
				m_bToBeErased = true;
				return;
			}
		}

		pCograph->ClearConnections(this);
		pSpanTree->ClearConnections(this);
		pMap->EraseKeyFrame(this);
		pKFDB->erase(this);
	}

	void KeyFrame::SetBad(KeyFrame *pPKF) {
		unique_lock<mutex> lock(m_MutexBad);
		m_bBad = true;
		m_Tcp = m_Transformation*pPKF->GetPoseInverse();
	}

	bool KeyFrame::isBad() {
		unique_lock<mutex> lock(m_MutexBad);
		return m_bBad;
	}

} // namespace SLAMRecon
