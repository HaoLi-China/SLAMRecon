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
#include "../ORB/ORBmatcher.h"
#include "Map.h"

using namespace std;

namespace SLAMRecon {
	
	long unsigned int MapPoint::n_MPNextId = 0;
	mutex MapPoint::m_GlobalMutex;

	MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap)
		:m_nObs(0), m_fMinDistance(0), m_fMaxDistance(0), m_pRefKF(pRefKF), 
		m_nVisible(1), m_nFound(1), m_bBad(false), m_pReplaced(static_cast<MapPoint*>(NULL)),
		m_nLastFrameSeen(0), m_nTrackReferenceForFrame(0), m_nFirstKFid(pRefKF->m_nKFId), m_nFuseCandidateForKF(0),
		m_pMap(pMap)
	{
		m_nMPId = n_MPNextId++;
		
		Pos.copyTo(m_WorldPos); 
		m_NormalVector = cv::Mat::zeros(3, 1, CV_32F);
	}

	MapPoint::MapPoint(const cv::Mat &Pos, Frame* pFrame, const int &idxF, Map* pMap)
		:m_nObs(0), m_fMinDistance(0), m_fMaxDistance(0), m_pRefKF(static_cast<KeyFrame*>(NULL)),
		m_nVisible(1), m_nFound(1), m_bBad(false), m_pReplaced(static_cast<MapPoint*>(NULL)),
		m_nLastFrameSeen(0), m_nTrackReferenceForFrame(0), m_nFirstKFid(-1), m_nFuseCandidateForKF(0),
		m_pMap(pMap)
	{
		m_nMPId = n_MPNextId++;

		Pos.copyTo(m_WorldPos);
		 
		cv::Mat Ow = pFrame->GetCameraCenter();
		m_NormalVector = m_WorldPos - Ow;
		m_NormalVector = m_NormalVector / cv::norm(m_NormalVector);
		 
		cv::Mat PC = Pos - Ow;
		const float dist = cv::norm(PC);
		const int level = pFrame->m_vKeysUn[idxF].octave;   
		const float levelScaleFactor = pFrame->m_pPLevelInfo->m_vScaleFactors[level];
		const int nLevels = pFrame->m_pPLevelInfo->m_nScaleLevels;

		m_fMaxDistance = dist*levelScaleFactor;
		m_fMinDistance = m_fMaxDistance / pFrame->m_pPLevelInfo->m_vScaleFactors[nLevels - 1];

		pFrame->m_Descriptors.row(idxF).copyTo(m_Descriptor);

	}

	MapPoint::~MapPoint() {

	}

	void MapPoint::SetWorldPos(const cv::Mat &Pos) {
		unique_lock<mutex> lock(m_MutexPos);
		Pos.copyTo(m_WorldPos);
	}

	cv::Mat MapPoint::GetWorldPos() {
		unique_lock<mutex> lock(m_MutexPos);
		return m_WorldPos.clone();
	}

	cv::Mat MapPoint::GetNormal() {
		unique_lock<mutex> lock(m_MutexPos); 
		return m_NormalVector.clone();
	}

	float MapPoint::GetMinDistanceInvariance() {
		unique_lock<mutex> lock(m_MutexPos);
		return 0.8f*m_fMinDistance;
	}

	float MapPoint::GetMaxDistanceInvariance() {
		unique_lock<mutex> lock(m_MutexPos);
		return 1.2f*m_fMaxDistance;
	}

	cv::Mat MapPoint::GetDescriptor() {
		unique_lock<mutex> lock(m_MutexObservations);
		return m_Descriptor.clone();
	}

	KeyFrame* MapPoint::GetReferenceKeyFrame() {
		unique_lock<mutex> lock(m_MutexObservations);
		return m_pRefKF;
	}

	map<KeyFrame*, size_t> MapPoint::GetObservations() {
		unique_lock<mutex> lock(m_MutexObservations);
		return m_Observations;
	}

	int MapPoint::Observations() {
		unique_lock<mutex> lock(m_MutexObservations);
		return m_nObs;
	}

	void MapPoint::IncreaseVisible(int n) {
		unique_lock<mutex> lock(m_MutexObservations);
		m_nVisible += n;
	}

	void MapPoint::IncreaseFound(int n) {
		unique_lock<mutex> lock(m_MutexObservations);
		m_nFound += n;
	}

	float MapPoint::GetFoundRatio(){
		unique_lock<mutex> lock(m_MutexObservations);
		return static_cast<float>(m_nFound) / m_nVisible;
	}

	int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF) {
		unique_lock<mutex> lock(m_MutexObservations);
		if (m_Observations.count(pKF))
			return m_Observations[pKF];
		else
			return -1;
	}

	bool MapPoint::IsInKeyFrame(KeyFrame *pKF) {
		unique_lock<mutex> lock(m_MutexObservations);
		return (m_Observations.count(pKF));
	}

	MapPoint* MapPoint::GetReplaced() {
		unique_lock<mutex> lock1(m_MutexObservations);
		unique_lock<mutex> lock2(m_MutexPos);
		return m_pReplaced;
	}

	void MapPoint::Replace(MapPoint* pMP) {

		if (pMP->m_nMPId == this->m_nMPId)
			return;
		 
		int nvisible, nfound;
		map<KeyFrame*, size_t> obs;
		{
			unique_lock<mutex> lock1(m_MutexObservations);
			obs = m_Observations;
			m_Observations.clear();
			m_bBad = true;
			nvisible = m_nVisible;
			nfound = m_nFound;
			m_pReplaced = pMP;
		}

		for (map<KeyFrame*, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++) {
			 
			KeyFrame* pKF = mit->first;
			 
			if (!pMP->IsInKeyFrame(pKF)) {
				pKF->ReplaceMapPoint(mit->second, pMP);
				pMP->AddObservation(pKF, mit->second);
			} 
			else {
				pKF->EraseMapPoint(mit->second);
			}
		}
		pMP->IncreaseFound(nfound);
		pMP->IncreaseVisible(nvisible);
		pMP->ComputeDistinctiveDescriptors();

		m_pMap->EraseMapPoint(this);
	}

	void MapPoint::AddObservation(KeyFrame* pKF, size_t idx) {
		unique_lock<mutex> lock(m_MutexObservations);
		if (m_Observations.count(pKF))
			return;
		m_Observations[pKF] = idx;
		if (pKF->m_vuRight[idx] >= 0)
			m_nObs += 2;
		else
			m_nObs++;
	}

	void MapPoint::EraseObservation(KeyFrame* pKF) {
		
		bool bBad = false;
		{
			unique_lock<mutex> lock(m_MutexObservations);
			if (m_Observations.count(pKF)) {
				int idx = m_Observations[pKF];
				if (pKF->m_vuRight[idx] >= 0)
					m_nObs -= 2;
				else
					m_nObs--;
				m_Observations.erase(pKF);

				if (m_pRefKF == pKF)
					m_pRefKF = m_Observations.begin()->first;
				 
				if (m_nObs <= 2)
					bBad = true;
			}
		}
       
		if (bBad)
			SetBadFlag();
	}

	void MapPoint::SetBadFlag() {
		map<KeyFrame*, size_t> obs;
		{ 
			unique_lock<mutex> lock1(m_MutexObservations);
			unique_lock<mutex> lock2(m_MutexPos);
			m_bBad = true;
			obs = m_Observations;
			m_Observations.clear();
		} 
		for (map<KeyFrame*, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++) {
			KeyFrame* pKF = mit->first;
			pKF->EraseMapPoint(mit->second);
		}

		m_pMap->EraseMapPoint(this);  
	}

	bool MapPoint::isBad() {
		unique_lock<mutex> lock1(m_MutexObservations);
		unique_lock<mutex> lock2(m_MutexPos);
		return m_bBad;
	}

	void MapPoint::ComputeDistinctiveDescriptors() { 
		vector<cv::Mat> vDescriptors;

		map<KeyFrame*, size_t> observations;
		{ 
			unique_lock<mutex> lock1(m_MutexObservations);
			if (m_bBad)
				return;
			observations = m_Observations;
		}
		
		if (observations.empty())
			return;
		 
		vDescriptors.reserve(observations.size());
		for (map<KeyFrame*, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
			KeyFrame* pKF = mit->first;
			vDescriptors.push_back(pKF->m_Descriptors.row(mit->second));
		}

		if (vDescriptors.empty())
			return; 

		const size_t N = vDescriptors.size();

		//float Distances[N][N] = {0};
		float** Distances;
		Distances = (float**)malloc(N * sizeof(float*));
		for (int i = 0; i < N; i++) 
			Distances[i] = (float*)malloc(N * sizeof(float));
		
		for (size_t i = 0; i < N; i++) {
			Distances[i][i] = 0;
			for (size_t j = i + 1; j < N; j++) {
				int distij = ORBmatcher::DescriptorDistance(vDescriptors[i], vDescriptors[j]);  
				Distances[i][j] = distij;
				Distances[j][i] = distij;
			}
		}
		 
		int BestMedian = INT_MAX;
		int BestIdx = 0;
		for (size_t i = 0; i < N; i++) {
			vector<int> vDists(Distances[i], Distances[i] + N);
			sort(vDists.begin(), vDists.end());
			int median = vDists[0.5*(N - 1)];

			if (median < BestMedian) {
				BestMedian = median;
				BestIdx = i;
			}
		}
		 
		for (int i = 0; i < N; i++)
			free(Distances[i]);
		free(Distances);

		{ 
			unique_lock<mutex> lock2(m_MutexObservations);
			m_Descriptor = vDescriptors[BestIdx].clone();
		}
	
	}

	void MapPoint::UpdateNormalAndDepth()
	{
		map<KeyFrame*, size_t> observations;
		KeyFrame* pRefKF; 
		cv::Mat Pos; 
		{ 
			unique_lock<mutex> lock1(m_MutexObservations);
			unique_lock<mutex> lock2(m_MutexPos);
			if (m_bBad)
				return;
			observations = m_Observations;
			pRefKF = m_pRefKF;
			Pos = m_WorldPos.clone();
		}
		
		if (observations.empty())
			return;

		cv::Mat normal = cv::Mat::zeros(3, 1, CV_32F);
		int n = 0; 
		for (map<KeyFrame*, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
			KeyFrame* pKF = mit->first;
			cv::Mat Owi = pKF->GetCameraCenter();
			cv::Mat normali = m_WorldPos - Owi;
			normal = normal + normali / cv::norm(normali);
			n++;
		}
		 
		cv::Mat PC = Pos - pRefKF->GetCameraCenter();
		const float dist = cv::norm(PC);
		const int level = pRefKF->m_vKeysUn[observations[pRefKF]].octave;
		const float levelScaleFactor = pRefKF->m_pPLevelInfo->m_vScaleFactors[level];
		const int nLevels = pRefKF->m_pPLevelInfo->m_nScaleLevels;

		{ 
			unique_lock<mutex> lock3(m_MutexPos);

			m_NormalVector = normal / n;  
			m_NormalVector = m_NormalVector / cv::norm(m_NormalVector);

			m_fMaxDistance = dist*levelScaleFactor;
			m_fMinDistance = m_fMaxDistance / pRefKF->m_pPLevelInfo->m_vScaleFactors[nLevels - 1];
		}
	}

	int MapPoint::PredictScale(const float &currentDist, const float &logScaleFactor) {
		float ratio;
		ratio = m_fMaxDistance / currentDist;
		int level = ceil(log(ratio) / logScaleFactor); 
		if (level >= 8)
			return 8;
		return ceil(log(ratio) / logScaleFactor);
	}
} // namespace SLAMRecon
