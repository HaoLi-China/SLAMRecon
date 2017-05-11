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

#include "Map.h"

using namespace std;
using namespace cv;
namespace SLAMRecon {
	

	Map::Map():
		m_nMaxKFid(0){

	}

	Map::~Map() {
		for (set<MapPoint*>::iterator sit = m_spMapPoints.begin(), send = m_spMapPoints.end(); sit != send; sit++)
			delete (*sit);

		for (set<Frame*>::iterator sit = m_spFrames.begin(), send = m_spFrames.end(); sit != send; sit++)
			delete (*sit);

		for (set<KeyFrame*>::iterator sit = m_spKeyFrames.begin(), send = m_spKeyFrames.end(); sit != send; sit++)
			delete *sit;
	}

	void Map::AddKeyFrame(KeyFrame *pKF) {
		unique_lock<mutex> lock(m_MutexMap);
		m_spKeyFrames.insert(pKF);
		if (pKF->m_nKFId > m_nMaxKFid)
			m_nMaxKFid = pKF->m_nKFId;
	}

	void Map::AddMapPoint(MapPoint *pMP) {
		unique_lock<mutex> lock(m_MutexMap);
		m_spMapPoints.insert(pMP);
	}

	void Map::EraseKeyFrame(KeyFrame *pKF) {
		unique_lock<mutex> lock(m_MutexMap);
		m_spKeyFrames.erase(pKF);
	}

	void Map::EraseMapPoint(MapPoint *pMP) {
		unique_lock<mutex> lock(m_MutexMap);
		m_spMapPoints.erase(pMP);
	}

	vector<KeyFrame*> Map::GetAllKeyFrames() {
		unique_lock<mutex> lock(m_MutexMap);
		return vector<KeyFrame*>(m_spKeyFrames.begin(), m_spKeyFrames.end());
	}

	vector<MapPoint*> Map::GetAllMapPoints() {
		unique_lock<mutex> lock(m_MutexMap);
		return vector<MapPoint*>(m_spMapPoints.begin(), m_spMapPoints.end());
	}

	long unsigned int Map::MapPointsInMap() {
		unique_lock<mutex> lock(m_MutexMap);
		return m_spMapPoints.size();
	}

	long unsigned int Map::KeyFramesInMap() {
		unique_lock<mutex> lock(m_MutexMap);
		return m_spKeyFrames.size();
	}
	  
	void Map::clear() { 
		for (set<MapPoint*>::iterator sit = m_spMapPoints.begin(), send = m_spMapPoints.end(); sit != send; sit++)
			delete *sit;

		for (set<KeyFrame*>::iterator sit = m_spKeyFrames.begin(), send = m_spKeyFrames.end(); sit != send; sit++)
			delete *sit;

		m_spMapPoints.clear();
		m_spKeyFrames.clear();

		m_vpReferenceMapPoints.clear();
		m_vpKeyFrameOrigins.clear();

		m_nMaxKFid = 0;


		//
		m_lRelativeFramePoses.clear();
		m_lpReferences.clear();
		m_lbLost.clear();
		m_lbKF.clear();
		m_lpFIdAndPoses.clear();
		m_modifiedKeyFrames.clear();
		m_CurFramePose = Mat();
	}

	long unsigned int Map::GetMaxKFid() {
		unique_lock<mutex> lock(m_MutexMap);
		return m_nMaxKFid;
	}

	void Map::AddFrame(Frame* pFrame) {
		//m_spFrames.insert(pFrame);
	}

	vector<Frame*> Map::GetAllFrames() {
		return vector<Frame*>(m_spFrames.begin(), m_spFrames.end());
	}


	void Map::addIdAndPose(int fId, Mat pose) {
		unique_lock<mutex> lock(m_MutexFIdAndPoses);
		pair<int, Mat> fIdAndPose(fId, pose);
		m_lpFIdAndPoses.push_back(fIdAndPose);
	}
	pair<int, Mat>  Map::getIdAndPose(){
		unique_lock<mutex> lock(m_MutexFIdAndPoses);
		pair<int, Mat> fIdAndPose(-1, cv::Mat::eye(0, 0, CV_32F));
		if (m_lpFIdAndPoses.empty())
			return fIdAndPose;
		fIdAndPose = m_lpFIdAndPoses.front();
		if (fIdAndPose.first == 0)
			cout << m_lpFIdAndPoses.size() << endl;
		m_lpFIdAndPoses.pop_front();
		
		return fIdAndPose;
	}

	void Map::addModifiedKeyFrame(KeyFrame* pKF) {
		unique_lock<mutex> lock(m_MutexModifiedKeyFrames);
		for (list<KeyFrame*>::iterator lit = m_modifiedKeyFrames.begin(); lit != m_modifiedKeyFrames.end();) {
			if ((*lit)->m_nKFId == pKF->m_nKFId){
				lit = m_modifiedKeyFrames.erase(lit); 
			}
			else{
				++lit;
			}
		}
		m_modifiedKeyFrames.push_back(pKF);
	}


	KeyFrame* Map::getModifiedKeyFrame() {
		unique_lock<mutex> lock(m_MutexModifiedKeyFrames);
		if (m_modifiedKeyFrames.empty())
			return NULL;
		KeyFrame* pKF = m_modifiedKeyFrames.front();
		m_modifiedKeyFrames.pop_front();
		return pKF;
	}


	void Map::addRelativeInfo(Mat RelativeFramePose, KeyFrame* pReferenceKF, bool bLost, bool bKF) {
		unique_lock<mutex> lock(m_MutexRelativeInfo);
		m_lRelativeFramePoses.push_back(RelativeFramePose);
		m_lpReferences.push_back(pReferenceKF);
		m_lbLost.push_back(bLost);
		m_lbKF.push_back(bKF);
	}

	void Map::addLastRelativeInfo(bool bLost) {
		unique_lock<mutex> lock(m_MutexRelativeInfo);
		m_lRelativeFramePoses.push_back(m_lRelativeFramePoses.back());
		m_lpReferences.push_back(m_lpReferences.back());
		m_lbLost.push_back(bLost);
		m_lbKF.push_back(m_lbKF.back());
	}

	std::list<std::pair<int, Mat> > Map::getFramesByKF(KeyFrame* pKF, SpanningTree *pSpanTree) {
		unique_lock<mutex> lock(m_MutexRelativeInfo);

		list<std::pair<int, Mat> > lIdPoses;

		list<KeyFrame*>::iterator lRit = m_lpReferences.begin();
		list<bool>::iterator lbKF = m_lbKF.begin();

		int idCount = 0;
		for (list<Mat>::iterator lit = m_lRelativeFramePoses.begin(), lend = m_lRelativeFramePoses.end(); lit != lend; lit++, lRit++, lbKF++) {

			KeyFrame* pKF2 = *lRit;
			if (pKF2->m_nKFId == pKF->m_nKFId) {
				pair<int, Mat> pairtemp(idCount, (*lit)); 
				lIdPoses.push_back(pairtemp);
			}
			idCount++;
		}
		return lIdPoses;
	}

	void Map::setCurFramePose(cv::Mat curFramePose) {
		unique_lock<mutex> lock(m_MutexCurFramePose);
		if (!curFramePose.empty())
			m_CurFramePose = curFramePose.clone();
	}
	cv::Mat Map::getCurFramePose() {
		unique_lock<mutex> lock(m_MutexCurFramePose);
		return m_CurFramePose.clone();
	}

} // namespace SLAMRecon


