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

#include "LoopClosing.h"
#include "Sim3Solver.h"
#include "Converter.h"
#include "Optimizer.h"

namespace SLAMRecon {

	LoopClosing::LoopClosing(Map *pMap, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, CovisibilityGraph* pCoGraph, SpanningTree* pSpanTree)
		:m_pMap(pMap), m_pKeyFrameDB(pDB), m_pORBVocabulary(pVoc),
		m_pCoGraph(pCoGraph), m_pSpanTree(pSpanTree),
		m_bFinishRequested(false), m_bFinished(true), m_LastLoopKFid(0),
		m_bRunningGBA(false), m_bFinishedGBA(true), m_bStopGBA(false)
	{
		m_nCovisibilityConsistencyTh = 3;
	}

	LoopClosing::~LoopClosing() {

	}

	void LoopClosing::SetTracker(Tracking *pTracker) {
		m_pTracker = pTracker;
	}

	void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper) {
		m_pLocalMapper = pLocalMapper;
	}

	void LoopClosing::Run() {
		m_bFinished = false;

		while (1) { 
			if (CheckNewKeyFrames()) { 
				 
				if (DetectLoop()) {
					 
					if (ComputeSim3()){
						// Perform loop fusion and pose graph optimization
						CorrectLoop();
					}
				}
				cout << "After LoopClosing, Current KeyFrame Id: " << m_pCurrentKF->m_nKFId << endl << endl;
			} 
			ResetIfRequested();
			 
			if (CheckFinish())
				break;

			Sleep(5);
		}
		 
		SetFinish();
	}

	void LoopClosing::InsertKeyFrame(KeyFrame *pKF) {
		unique_lock<mutex> lock(m_MutexLoopQueue);
		if (pKF->m_nKFId != 0)
			m_lpLoopKeyFrameQueue.push_back(pKF);
	}

	bool LoopClosing::CheckNewKeyFrames() {
		unique_lock<mutex> lock(m_MutexLoopQueue);
		return(!m_lpLoopKeyFrameQueue.empty());
	}

	bool LoopClosing::DetectLoop() {

		{
			unique_lock<mutex> lock(m_MutexLoopQueue);
			m_pCurrentKF = m_lpLoopKeyFrameQueue.front();
			m_lpLoopKeyFrameQueue.pop_front();

			cout << "Before LoopClosing, Current KeyFrame Id: " << m_pCurrentKF->m_nKFId << endl;

			// Avoid that a keyframe can be erased while it is being process by this thread, 
			m_pCurrentKF->SetNotErase();
		}
		 
		if (m_pCurrentKF->m_nKFId < m_LastLoopKFid + 10) {
			m_pKeyFrameDB->add(m_pCurrentKF);
			m_pCurrentKF->SetErase(m_pCoGraph, m_pSpanTree, m_pKeyFrameDB, m_pMap);  
			return false;
		}
		 
		const vector<KeyFrame*> vpConnectedKeyFrames = m_pCoGraph->GetVectorCovisibleKeyFrames(m_pCurrentKF);
		const DBoW2::BowVector &CurrentBowVec = m_pCurrentKF->m_BowVec;
		float minScore = 1;
		for (size_t i = 0; i < vpConnectedKeyFrames.size(); i++) {
			KeyFrame* pKF = vpConnectedKeyFrames[i];
			if (pKF->isBad())
				continue;
			const DBoW2::BowVector &BowVec = pKF->m_BowVec;

			float score = m_pORBVocabulary->score(CurrentBowVec, BowVec);
			if (score < minScore)
				minScore = score;
		}
		 
		vector<KeyFrame*> vpCandidateKFs = m_pKeyFrameDB->DetectLoopCandidates(m_pCurrentKF, minScore);
		 
		if (vpCandidateKFs.empty()) {
			m_pKeyFrameDB->add(m_pCurrentKF);
			m_vConsistentGroups.clear();
			m_pCurrentKF->SetErase(m_pCoGraph, m_pSpanTree, m_pKeyFrameDB, m_pMap);  
			return false;
		}
		 
		m_vpEnoughConsistentCandidates.clear();

		vector<ConsistentGroup> vCurrentConsistentGroups;  
		vector<bool> vbConsistentGroup(m_vConsistentGroups.size(), false);  
		  
		for (size_t i = 0, iend = vpCandidateKFs.size(); i < iend; i++) {

			KeyFrame* pCandidateKF = vpCandidateKFs[i];

			set<KeyFrame*> spCandidateGroup = m_pCoGraph->GetConnectedKeyFrames(pCandidateKF);
			spCandidateGroup.insert(pCandidateKF);

			bool bEnoughConsistent = false;
			bool bConsistentForSomeGroup = false;
			 
			for (size_t iG = 0, iendG = m_vConsistentGroups.size(); iG < iendG; iG++) {
				set<KeyFrame*> sPreviousGroup = m_vConsistentGroups[iG].first;
				 
				bool bConsistent = false;
				for (set<KeyFrame*>::iterator sit = spCandidateGroup.begin(), send = spCandidateGroup.end(); sit != send; sit++) {
					if (sPreviousGroup.count(*sit)) {
						bConsistent = true;
						bConsistentForSomeGroup = true;
						break;
					}
				}
				 
				if (bConsistent) {

					int nPreviousConsistency = m_vConsistentGroups[iG].second;
					int nCurrentConsistency = nPreviousConsistency + 1;  
					if (!vbConsistentGroup[iG]) {
						ConsistentGroup cg = make_pair(spCandidateGroup, nCurrentConsistency);
						vCurrentConsistentGroups.push_back(cg);
						vbConsistentGroup[iG] = true; //this avoid to include the same group more than once
					}
					 
					if (nCurrentConsistency >= m_nCovisibilityConsistencyTh && !bEnoughConsistent) {
						m_vpEnoughConsistentCandidates.push_back(pCandidateKF);
						bEnoughConsistent = true; 
					}
				}
			}
			 
			if (!bConsistentForSomeGroup) {
				ConsistentGroup cg = make_pair(spCandidateGroup, 0);
				vCurrentConsistentGroups.push_back(cg);
			}
		}
		 
		m_vConsistentGroups = vCurrentConsistentGroups;
		 
		m_pKeyFrameDB->add(m_pCurrentKF);

		if (m_vpEnoughConsistentCandidates.empty()) {
			m_pCurrentKF->SetErase(m_pCoGraph, m_pSpanTree, m_pKeyFrameDB, m_pMap);
			return false;
		}
		else {
			m_pCurrentKF->SetErase(m_pCoGraph, m_pSpanTree, m_pKeyFrameDB, m_pMap);
			return true;
		}
	}

	bool LoopClosing::ComputeSim3() {
		 
		const int nInitialCandidates = m_vpEnoughConsistentCandidates.size();
		 
		ORBmatcher matcher(0.75, true);
		 
		vector<Sim3Solver*> vpSim3Solvers;
		vpSim3Solvers.resize(nInitialCandidates);
		 
		vector<vector<MapPoint*> > vvpMapPointMatches;
		vvpMapPointMatches.resize(nInitialCandidates);
		 
		vector<bool> vbDiscarded;
		vbDiscarded.resize(nInitialCandidates);
		 
		int nCandidates = 0; 

		for (int i = 0; i < nInitialCandidates; i++) {
			 
			KeyFrame* pKF = m_vpEnoughConsistentCandidates[i];
			 
			// avoid that local mapping erase it while it is being processed in this thread
			pKF->SetNotErase();

			if (pKF->isBad()) {
				vbDiscarded[i] = true;
				continue;
			}
			 
			int nmatches = matcher.SearchByBoW(m_pCurrentKF, pKF, vvpMapPointMatches[i]);
			 
			if (nmatches < 20) {
				vbDiscarded[i] = true;
				continue;
			}
			else {
				// mbFixScale    mSensor!=MONOCULAR , true
				// Sim3Solver* pSolver = new Sim3Solver(m_pCurrentKF, pKF, vvpMapPointMatches[i], mbFixScale);
				Sim3Solver* pSolver = new Sim3Solver(m_pCurrentKF, pKF, vvpMapPointMatches[i]);
				pSolver->SetRansacParameters(0.99, 20, 300);
				vpSim3Solvers[i] = pSolver;
			}
			nCandidates++;
		}

		bool bMatch = false;
		 
		while (nCandidates > 0 && !bMatch) {

			for (int i = 0; i < nInitialCandidates; i++) {

				if (vbDiscarded[i])
					continue;

				KeyFrame* pKF = m_vpEnoughConsistentCandidates[i];
				 
				vector<bool> vbInliers;  
				int nInliers; 
				bool bNoMore;  
				Sim3Solver* pSolver = vpSim3Solvers[i];
				cv::Mat Scm = pSolver->iterate(5, bNoMore, vbInliers, nInliers);
				 
				if (bNoMore) {
					vbDiscarded[i] = true;
					nCandidates--;
				}

				// If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences 
				if (!Scm.empty()) {
					 
					vector<MapPoint*> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPoint*>(NULL));
					for (size_t j = 0, jend = vbInliers.size(); j < jend; j++) {
						if (vbInliers[j])
							vpMapPointMatches[j] = vvpMapPointMatches[i][j];
					}


					cv::Mat R12 = pSolver->GetEstimatedRotation();
					cv::Mat t12 = pSolver->GetEstimatedTranslation();

					matcher.SearchBySim3(m_pCurrentKF, pKF, vpMapPointMatches, R12, t12, 7.5);

					g2o::Sim3 gScm(Converter::toMatrix3d(R12), Converter::toVector3d(t12), 1.0);

					const int nInliers = Optimizer::OptimizeSim3(m_pCurrentKF, pKF, vpMapPointMatches, gScm, 10);

					if (nInliers >= 20) {
						bMatch = true;
						m_pMatchedKF = pKF;
						g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()), Converter::toVector3d(pKF->GetTranslation()), 1.0);
						m_g2oScw = gScm*gSmw;
						m_Scw = Converter::toCvMat(m_g2oScw); 
						m_vpCurrentMatchedPoints = vpMapPointMatches;
						break;
					}
				}
			}
		}

		if (!bMatch) {
			for (int i = 0; i < nInitialCandidates; i++)
				m_vpEnoughConsistentCandidates[i]->SetErase(m_pCoGraph, m_pSpanTree, m_pKeyFrameDB, m_pMap);
			m_pCurrentKF->SetErase(m_pCoGraph, m_pSpanTree, m_pKeyFrameDB, m_pMap);
			return false;
		}

		// Retrieve MapPoints seen in Loop Keyframe and neighbors
		vector<KeyFrame*> vpLoopConnectedKFs = m_pCoGraph->GetVectorCovisibleKeyFrames(m_pMatchedKF);
		vpLoopConnectedKFs.push_back(m_pMatchedKF);
		m_vpLoopMapPoints.clear();
		for (vector<KeyFrame*>::iterator vit = vpLoopConnectedKFs.begin(); vit != vpLoopConnectedKFs.end(); vit++) {
			KeyFrame* pKF = *vit;
			vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
			for (size_t i = 0, iend = vpMapPoints.size(); i < iend; i++) {
				MapPoint* pMP = vpMapPoints[i];
				if (pMP) {
					if (!pMP->isBad() && pMP->m_nLoopPointForKF != m_pCurrentKF->m_nKFId) {
						m_vpLoopMapPoints.push_back(pMP);
						pMP->m_nLoopPointForKF = m_pCurrentKF->m_nKFId;
					}
				}
			}
		}

		matcher.SearchByProjection(m_pCurrentKF, m_Scw, m_vpLoopMapPoints, m_vpCurrentMatchedPoints, 10);

		// If enough matches accept Loop
		int nTotalMatches = 0;
		for (size_t i = 0; i < m_vpCurrentMatchedPoints.size(); i++) {
			if (m_vpCurrentMatchedPoints[i])
				nTotalMatches++;
		}

		if (nTotalMatches >= 40) {
			for (int i = 0; i < nInitialCandidates; i++)
				if (m_vpEnoughConsistentCandidates[i] != m_pMatchedKF)
					m_vpEnoughConsistentCandidates[i]->SetErase(m_pCoGraph, m_pSpanTree, m_pKeyFrameDB, m_pMap);
			return true;
		}
		else {
			for (int i = 0; i < nInitialCandidates; i++)
				m_vpEnoughConsistentCandidates[i]->SetErase(m_pCoGraph, m_pSpanTree, m_pKeyFrameDB, m_pMap);
			m_pCurrentKF->SetErase(m_pCoGraph, m_pSpanTree, m_pKeyFrameDB, m_pMap);
			return false;
		}

	}

	void LoopClosing::CorrectLoop() {

		cout << "Loop detected!" << endl;

		// Send a stop signal to Local Mapping
		// Avoid new keyframes are inserted while correcting the loop
		m_pLocalMapper->RequestStop();

		// If a Global Bundle Adjustment is running, abort it
		if (isRunningGBA()) {

			m_bStopGBA = true;

			while (!isFinishedGBA())
				Sleep(5);

			m_pThreadGBA->join();
			delete m_pThreadGBA;
		}

		while (!m_pLocalMapper->isStopped()) {
			Sleep(1);
		}

		m_pCoGraph->UpdateConnections(m_pCurrentKF);
		m_pSpanTree->UpdateConnections(m_pCurrentKF);

		m_vpCurrentConnectedKFs = m_pCoGraph->GetVectorCovisibleKeyFrames(m_pCurrentKF);
		m_vpCurrentConnectedKFs.push_back(m_pCurrentKF);

		KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
		CorrectedSim3[m_pCurrentKF] = m_g2oScw;
		cv::Mat Twc = m_pCurrentKF->GetPoseInverse();

		{
			// Get Map Mutex
			unique_lock<mutex> lock(m_pMap->m_MutexMapUpdate);

			for (vector<KeyFrame*>::iterator vit = m_vpCurrentConnectedKFs.begin(), vend = m_vpCurrentConnectedKFs.end(); vit != vend; vit++) {

				KeyFrame* pKFi = *vit;

				cv::Mat Tiw = pKFi->GetPose();

				if (pKFi != m_pCurrentKF) { 
					cv::Mat Tic = Tiw*Twc;
					cv::Mat Ric = Tic.rowRange(0, 3).colRange(0, 3);
					cv::Mat tic = Tic.rowRange(0, 3).col(3);
					g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric), Converter::toVector3d(tic), 1.0);
					g2o::Sim3 g2oCorrectedSiw = g2oSic*m_g2oScw;
					CorrectedSim3[pKFi] = g2oCorrectedSiw;
				}

				cv::Mat Riw = Tiw.rowRange(0, 3).colRange(0, 3);
				cv::Mat tiw = Tiw.rowRange(0, 3).col(3);
				g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw), Converter::toVector3d(tiw), 1.0);
				NonCorrectedSim3[pKFi] = g2oSiw;
			}

			for (KeyFrameAndPose::iterator mit = CorrectedSim3.begin(), mend = CorrectedSim3.end(); mit != mend; mit++) {

				KeyFrame* pKFi = mit->first;
				g2o::Sim3 g2oCorrectedSiw = mit->second;
				g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

				g2o::Sim3 g2oSiw = NonCorrectedSim3[pKFi];

				vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
				for (size_t iMP = 0, endMPi = vpMPsi.size(); iMP < endMPi; iMP++) {
					MapPoint* pMPi = vpMPsi[iMP];
					if (!pMPi)
						continue;
					if (pMPi->isBad())
						continue;
					if (pMPi->m_nCorrectedByKF == m_pCurrentKF->m_nKFId)
						continue;

					cv::Mat P3Dw = pMPi->GetWorldPos();
					Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(P3Dw);
					Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

					cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
					pMPi->SetWorldPos(cvCorrectedP3Dw);
					pMPi->m_nCorrectedByKF = m_pCurrentKF->m_nKFId;
					pMPi->m_nCorrectedReference = pKFi->m_nKFId;
					pMPi->UpdateNormalAndDepth();
				}

				// Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
				Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
				Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
				cv::Mat correctedTiw = Converter::toCvSE3(eigR, eigt);
				pKFi->SetPose(correctedTiw);

				m_pMap->addModifiedKeyFrame(pKFi);

				m_pCoGraph->UpdateConnections(m_pCurrentKF);
				m_pSpanTree->UpdateConnections(m_pCurrentKF);
			}

			for (size_t i = 0; i < m_vpCurrentMatchedPoints.size(); i++) {
				if (m_vpCurrentMatchedPoints[i]) {
					MapPoint* pLoopMP = m_vpCurrentMatchedPoints[i];
					MapPoint* pCurMP = m_pCurrentKF->GetMapPoint(i);
					if (pCurMP)
						pCurMP->Replace(pLoopMP);
					else {
						m_pCurrentKF->AddMapPoint(pLoopMP, i);
						pLoopMP->AddObservation(m_pCurrentKF, i);
						pLoopMP->ComputeDistinctiveDescriptors();
					}
				}
			}

		}

		SearchAndFuse(CorrectedSim3);

		map<KeyFrame*, set<KeyFrame*> > LoopConnections;

		for (vector<KeyFrame*>::iterator vit = m_vpCurrentConnectedKFs.begin(), vend = m_vpCurrentConnectedKFs.end(); vit != vend; vit++) {
			KeyFrame* pKFi = *vit;

			vector<KeyFrame*> vpPreviousNeighbors = m_pCoGraph->GetVectorCovisibleKeyFrames(pKFi);

			m_pCoGraph->UpdateConnections(pKFi);
			m_pSpanTree->UpdateConnections(pKFi);

			LoopConnections[pKFi] = m_pCoGraph->GetConnectedKeyFrames(pKFi);

			for (vector<KeyFrame*>::iterator vit_prev = vpPreviousNeighbors.begin(), vend_prev = vpPreviousNeighbors.end(); vit_prev != vend_prev; vit_prev++) {
				LoopConnections[pKFi].erase(*vit_prev);
			}
			for (vector<KeyFrame*>::iterator vit2 = m_vpCurrentConnectedKFs.begin(), vend2 = m_vpCurrentConnectedKFs.end(); vit2 != vend2; vit2++) {
				LoopConnections[pKFi].erase(*vit2);
			}
		}

		Optimizer::OptimizeEssentialGraph(m_pMap, m_pMatchedKF, m_pCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, m_pCoGraph, m_pSpanTree);

		m_pSpanTree->AddLoopEdge(m_pCurrentKF, m_pMatchedKF);

		m_bRunningGBA = true;
		m_bFinishedGBA = false;
		m_bStopGBA = false;

		m_pThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment, this, m_pCurrentKF->m_nKFId);

		m_pLocalMapper->Release();

		cout << "Loop Closed!" << endl;

		m_LastLoopKFid = m_pCurrentKF->m_nKFId;
	}

	void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap) {

		ORBmatcher matcher(0.8);

		for (KeyFrameAndPose::const_iterator mit = CorrectedPosesMap.begin(), mend = CorrectedPosesMap.end(); mit != mend; mit++) {

			KeyFrame* pKF = mit->first;

			g2o::Sim3 g2oScw = mit->second;
			cv::Mat cvScw = Converter::toCvMat(g2oScw);

			vector<MapPoint*> vpReplacePoints(m_vpLoopMapPoints.size(), static_cast<MapPoint*>(NULL));
			
			matcher.Fuse(pKF, cvScw, m_vpLoopMapPoints, 4, vpReplacePoints);

			unique_lock<mutex> lock(m_pMap->m_MutexMapUpdate);
			const int nLP = m_vpLoopMapPoints.size();
			for (int i = 0; i < nLP; i++) {
				MapPoint* pRep = vpReplacePoints[i];
				if (pRep) {
					pRep->Replace(m_vpLoopMapPoints[i]);
				}
			}
		}
	}

	void LoopClosing::RunGlobalBundleAdjustment(unsigned long nLoopKF) {

		cout << "Starting Global Bundle Adjustment" << endl;

		Optimizer::GlobalBundleAdjustemnt(m_pMap, 20, &m_bStopGBA, nLoopKF, false);

		{
			unique_lock<mutex> lock(m_MutexGBA);

			if (!m_bStopGBA) {

				cout << "Global Bundle Adjustment finished" << endl;
				cout << "Updating map ..." << endl;

				m_pLocalMapper->RequestStop();
				while (!m_pLocalMapper->isStopped() && !m_pLocalMapper->isFinished()) {
					Sleep(1);
				}

				unique_lock<mutex> lock(m_pMap->m_MutexMapUpdate);

				list<KeyFrame*> lpKFtoCheck(m_pMap->m_vpKeyFrameOrigins.begin(), m_pMap->m_vpKeyFrameOrigins.end());

				vector<KeyFrame*> allKeyFrames = m_pMap->GetAllKeyFrames();

				while (!lpKFtoCheck.empty()) {

					KeyFrame* pKF = lpKFtoCheck.front();
					cv::Mat Twc = pKF->GetPoseInverse();

					const set<KeyFrame*> sChilds = m_pSpanTree->GetChilds(pKF);

					for (set<KeyFrame*>::const_iterator sit = sChilds.begin(); sit != sChilds.end(); sit++) {
						KeyFrame* pChild = *sit;
						if (pChild->m_nBAGlobalForKF != nLoopKF) {
							cv::Mat Tchildc = pChild->GetPose()*Twc;
							pChild->m_TcwGBA = Tchildc*pKF->m_TcwGBA; 
							pChild->m_nBAGlobalForKF = nLoopKF;

						}
						lpKFtoCheck.push_back(pChild);
					}

					pKF->m_TcwBefGBA = pKF->GetPose();
					pKF->SetPose(pKF->m_TcwGBA);

					m_pMap->addModifiedKeyFrame(pKF);

					lpKFtoCheck.pop_front();
				}

				const vector<MapPoint*> vpMPs = m_pMap->GetAllMapPoints();

				for (size_t i = 0; i<vpMPs.size(); i++) {
					MapPoint* pMP = vpMPs[i];

					if (pMP->isBad())
						continue;
					
					if (pMP->m_nBAGlobalForKF == nLoopKF) {
						pMP->SetWorldPos(pMP->m_PosGBA);
					}
					else {

						KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();

						if (pRefKF->m_nBAGlobalForKF != nLoopKF)
							continue;

						cv::Mat Rcw = pRefKF->m_TcwBefGBA.rowRange(0, 3).colRange(0, 3);
						cv::Mat tcw = pRefKF->m_TcwBefGBA.rowRange(0, 3).col(3);
						cv::Mat Xc = Rcw*pMP->GetWorldPos() + tcw;

						// Backproject using corrected camera
						cv::Mat Twc = pRefKF->GetPoseInverse();
						cv::Mat Rwc = Twc.rowRange(0, 3).colRange(0, 3);
						cv::Mat twc = Twc.rowRange(0, 3).col(3);

						pMP->SetWorldPos(Rwc*Xc + twc);
					}
				}
				m_pLocalMapper->Release();

				cout << "Map updated!" << endl;
			}

			m_bFinishedGBA = true;
			m_bRunningGBA = false;
		}
	}

	void LoopClosing::RequestFinish() {
		unique_lock<mutex> lock(m_MutexFinish);
		m_bFinishRequested = true;
	}

	bool LoopClosing::CheckFinish(){
		unique_lock<mutex> lock(m_MutexFinish);
		return m_bFinishRequested;
	}

	void LoopClosing::SetFinish() {
		unique_lock<mutex> lock(m_MutexFinish);
		m_bFinished = true;
	}

	bool LoopClosing::isFinished() {
		unique_lock<mutex> lock(m_MutexFinish);
		return m_bFinished;
	}


	void LoopClosing::RequestReset() {
		{
			unique_lock<mutex> lock(m_MutexReset);
			m_bResetRequested = true;
		}

		while (1) {
			{
				unique_lock<mutex> lock2(m_MutexReset);
				if (!m_bResetRequested)
					break;
			}
			Sleep(5);
		}
	}

	void LoopClosing::ResetIfRequested() {
		unique_lock<mutex> lock(m_MutexReset);
		if (m_bResetRequested) {
			m_lpLoopKeyFrameQueue.clear();
			m_LastLoopKFid = 0;
			m_bResetRequested = false;
		}
	}

} // namespace SLAMRecon