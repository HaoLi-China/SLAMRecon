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

#include "Tracking.h"
#include "Optimizer.h"

namespace SLAMRecon {

	Tracking::Tracking(const string &strSettingPath, ORBVocabulary* voc, CovisibilityGraph* cograph, SpanningTree* spantree, KeyFrameDatabase* keyFrameDatabase, Map *pMap)
		: m_pORBVocabulary(voc), m_pCoGraph(cograph), m_pSpanTree(spantree), m_pKeyFrameDB(keyFrameDatabase), m_pMap(pMap), m_State(NO_IMAGES_YET),
		m_nLastRelocFrameId(0)
	{
		cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
		float fx = fSettings["Camera.fx"];
		float fy = fSettings["Camera.fy"];
		float cx = fSettings["Camera.cx"];
		float cy = fSettings["Camera.cy"];

		cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
		K.at<float>(0, 0) = fx;
		K.at<float>(1, 1) = fy;
		K.at<float>(0, 2) = cx;
		K.at<float>(1, 2) = cy;
		K.copyTo(m_K);

		cv::Mat DistCoef(4, 1, CV_32F);
		DistCoef.at<float>(0) = fSettings["Camera.k1"];
		DistCoef.at<float>(1) = fSettings["Camera.k2"];
		DistCoef.at<float>(2) = fSettings["Camera.p1"];
		DistCoef.at<float>(3) = fSettings["Camera.p2"];
		const float k3 = fSettings["Camera.k3"];
		if (k3 != 0) {
			DistCoef.resize(5);
			DistCoef.at<float>(4) = k3;
		}
		DistCoef.copyTo(m_DistCoef);

		cout << endl << "Camera Parameters: " << endl;
		cout << "- fx: " << fx << endl;
		cout << "- fy: " << fy << endl;
		cout << "- cx: " << cx << endl;
		cout << "- cy: " << cy << endl;
		cout << "- k1: " << DistCoef.at<float>(0) << endl;
		cout << "- k2: " << DistCoef.at<float>(1) << endl;
		if (DistCoef.rows == 5)
			cout << "- k3: " << DistCoef.at<float>(4) << endl;
		cout << "- p1: " << DistCoef.at<float>(2) << endl;
		cout << "- p2: " << DistCoef.at<float>(3) << endl;

		m_bf = fSettings["Camera.bf"];

		float fps = fSettings["Camera.fps"];
		if (fps == 0)
			fps = 30;
		m_MinFrames = 0;
		m_MaxFrames = fps;
		m_MaxFrames = 20;

		int nRGB = fSettings["Camera.RGB"];
		m_RGB = nRGB;

		if (m_RGB)
			cout << "- color order: RGB (ignored if grayscale)" << endl;
		else
			cout << "- color order: BGR (ignored if grayscale)" << endl;

		m_DepthMapFactor = fSettings["DepthMapFactor"];
		if (m_DepthMapFactor == 0)
			m_DepthMapFactor = 1;
		else
			m_DepthMapFactor = 1.0f / m_DepthMapFactor;

		m_fThDepth = m_bf * (float)fSettings["ThDepth"] / fx;
		cout << endl << "Depth Threshold (Close/Far Points): " << m_fThDepth << endl;

		int nFeatures = fSettings["ORBextractor.nFeatures"];
		float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
		int nLevels = fSettings["ORBextractor.nLevels"];
		int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
		int fMinThFAST = fSettings["ORBextractor.minThFAST"];

		cout << endl << "ORB Extractor Parameters: " << endl;
		cout << "- Number of Features: " << nFeatures << endl;
		cout << "- Scale Levels: " << nLevels << endl;
		cout << "- Scale Factor: " << fScaleFactor << endl;
		cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
		cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

		m_pORBextractor = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

	}

	Tracking::~Tracking() {
		if (m_pORBextractor != NULL)
			delete m_pORBextractor;
	}

	void Tracking::SetLocalMapper(LocalMapping *pLocalMapper) {
		m_pLocalMapper = pLocalMapper;
	}

	void Tracking::SetLoopCloser(LoopClosing *pLoopCloser) {
		m_pLoopCloser = pLoopCloser;
	}

	cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD) {
		m_ImageGray = imRGB;
		cv::Mat imDepth = imD;

		if (m_ImageGray.channels() == 3) {
			if (m_RGB)
				cv::cvtColor(m_ImageGray, m_ImageGray, CV_RGB2GRAY);
			else
				cv::cvtColor(m_ImageGray, m_ImageGray, CV_BGR2GRAY);
		}
		else if (m_ImageGray.channels() == 4) {
			if (m_RGB)
				cv::cvtColor(m_ImageGray, m_ImageGray, CV_RGBA2GRAY);
			else
				cv::cvtColor(m_ImageGray, m_ImageGray, CV_BGRA2GRAY);
		}

		if (m_DepthMapFactor != 1 || imDepth.type() != CV_32F);
			imDepth.convertTo(imDepth, CV_32F, m_DepthMapFactor);

		m_CurrentFrame = Frame(m_ImageGray, imDepth, m_pORBextractor, m_pORBVocabulary, m_K, m_DistCoef, m_bf, m_fThDepth);
		m_CurrentFrame.setRGBImg(imRGB);

		Track();
		
		return m_CurrentFrame.m_Transformation.clone();
	}

	void Tracking::Track() {
		
		if (m_State == NO_IMAGES_YET) { // First frame comes, the tracking is not initialized.
			m_State = NOT_INITIALIZED;
		}

		if (m_State == NOT_INITIALIZED) {
			Initialization();
			//cout << "After Initialization, m_State is " << m_State << endl;
			if (m_State != OK) 
				return;
		} else {

			bool bOK;

			if (m_State == OK) {

				if (m_Velocity.empty() || m_CurrentFrame.m_nFId < m_nLastRelocFrameId + 2) {
					bOK = TrackReferenceKeyFrame();
					//cout << "After TrackReferenceKeyFrame, bOK is " << bOK << endl;
				}
				else {
					bOK = TrackWithMotionModel();
					//cout << "After TrackWithMotionModel, bOK is " << bOK << endl;
					if (!bOK) {
						bOK = TrackReferenceKeyFrame();
						//cout << "After TrackWithMotionModel and TrackReferenceKeyFrame, bOK is " << bOK << endl;
					}
				}
			} else {
				// Global Relocalization
				bOK = Relocalization();
				cout << "Relocalization" << endl;
				//cout << "After Relocalization, bOK is " << bOK << endl;
			}

			m_CurrentFrame.m_pReferenceKF = m_pReferenceKF;

			if (bOK) {
				bOK = TrackLocalMap();
				//cout << "After TrackLocalMap, bOK is " << bOK << endl;
			}

			if (bOK)
				m_State = OK;
			else
				m_State = LOST;

			if (bOK) {

				if (!m_LastFrame.m_Transformation.empty()) {
					cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
					cv::Mat Rwc = m_LastFrame.GetRotation().t();
					Rwc.copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
					m_LastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
					m_Velocity = m_CurrentFrame.m_Transformation*LastTwc;
				}
				else
					m_Velocity = cv::Mat();

				for (int i = 0; i < m_CurrentFrame.m_nKeys; i++) {

					MapPoint* pMP = m_CurrentFrame.m_vpMapPoints[i];
					if (pMP)
						if (pMP->Observations() < 1) {
							m_CurrentFrame.m_vbOutlier[i] = false;
							m_CurrentFrame.m_vpMapPoints[i] = static_cast<MapPoint*>(NULL);
						}
				}

				for (list<MapPoint*>::iterator lit = m_lpTemporalPoints.begin(), lend = m_lpTemporalPoints.end(); lit != lend; lit++) {
					MapPoint* pMP = *lit;
					delete pMP;
				}
				m_lpTemporalPoints.clear();

				if (NeedNewKeyFrame()){
					CreateNewKeyFrame();
				}

				for (int i = 0; i < m_CurrentFrame.m_nKeys; i++) {
					if (m_CurrentFrame.m_vpMapPoints[i] && m_CurrentFrame.m_vbOutlier[i])
						m_CurrentFrame.m_vpMapPoints[i] = static_cast<MapPoint*>(NULL);
				}
			}

			if (m_State == LOST) {
				if (m_pMap->KeyFramesInMap() <= 5) {
					cout << "Track lost soon after initialisation, reseting..." << endl;
					Reset();
					return;
				}
			}

			if (!m_CurrentFrame.m_pReferenceKF)
				m_CurrentFrame.m_pReferenceKF = m_pReferenceKF;

			m_LastFrame = Frame(m_CurrentFrame);
		}

		if (m_State == OK) {
			cv::Mat p_Transformation = m_CurrentFrame.m_Transformation * m_CurrentFrame.m_pReferenceKF->GetPoseInverse();
			m_lRelativeFramePoses.push_back(p_Transformation);
			m_lpReferences.push_back(m_pReferenceKF);
			m_lbLost.push_back(m_State == LOST);

			if (m_CurrentFrame.m_pReferenceKF->m_nFId == m_CurrentFrame.m_nFId) {
				m_pMap->addRelativeInfo(p_Transformation, m_pReferenceKF, m_State == LOST, true);
				m_CurrentFrame.m_pReferenceKF->m_oldCameraPose = m_CurrentFrame.m_pReferenceKF->GetPose();
			}
			else
				m_pMap->addRelativeInfo(p_Transformation, m_pReferenceKF, m_State == LOST, false);
			
			//m_pMap->addIdAndPose(m_CurrentFrame.m_nFId, m_CurrentFrame.m_Transformation);
			m_pMap->addIdAndPose(m_CurrentFrame.m_nFId, m_CurrentFrame.m_Transformation.clone());

			m_pMap->setCurFramePose(m_CurrentFrame.m_Transformation);
		}
		else {
			m_lRelativeFramePoses.push_back(m_lRelativeFramePoses.back());
			m_lpReferences.push_back(m_lpReferences.back());
			m_lbLost.push_back(m_State == LOST);

			m_pMap->addLastRelativeInfo(m_State == LOST);

			//m_pMap->addIdAndPose(m_CurrentFrame.m_nFId, m_CurrentFrame.m_Transformation);
			//m_pMap->addIdAndPose(m_CurrentFrame.m_nFId, m_CurrentFrame.m_Transformation.clone());
		}
	}

	void Tracking::Initialization() {

		if (m_CurrentFrame.m_nKeys > 500) {

			m_CurrentFrame.SetPose(cv::Mat::eye(4, 4, CV_32F)); 
			
			KeyFrame* pKFini = new KeyFrame(m_CurrentFrame);
			
			m_pMap->AddKeyFrame(pKFini); 

			for (int i = 0; i<m_CurrentFrame.m_nKeys; i++) {
				float z = m_CurrentFrame.m_vfDepth[i];
				if (z>0) {
					cv::Mat x3D = m_CurrentFrame.ComputeWorldPos(i);
					MapPoint* pNewMP = new MapPoint(x3D, pKFini, m_pMap);
					pNewMP->AddObservation(pKFini, i);

					pNewMP->ComputeDistinctiveDescriptors();
					pNewMP->UpdateNormalAndDepth();

					pKFini->AddMapPoint(pNewMP, i);
					m_pMap->AddMapPoint(pNewMP);

					m_CurrentFrame.m_vpMapPoints[i] = pNewMP;
				}
			}
			
			cout << "New map created with " << m_pMap->MapPointsInMap() << " points" << endl;

			m_pLocalMapper->InsertKeyFrame(pKFini);

			m_CurrentFrame.m_pReferenceKF = pKFini;

			m_pReferenceKF = pKFini;
			m_vpLocalKeyFrames.push_back(pKFini);
			m_vpLocalMapPoints = m_pMap->GetAllMapPoints();
			
			m_LastFrame = Frame(m_CurrentFrame);
			m_pLastKeyFrame = pKFini;
			m_nLastKeyFrameId = m_CurrentFrame.m_nFId;

			m_pMap->m_vpKeyFrameOrigins.push_back(pKFini);

			m_State = OK;
			
		}
	}

	bool Tracking::TrackReferenceKeyFrame() {

		m_CurrentFrame.ComputeBoW();

		ORBmatcher matcher(0.7, true);
		vector<MapPoint*> vpMapPointMatches;

		int nmatches = matcher.SearchByBoW(m_pReferenceKF, m_CurrentFrame, vpMapPointMatches);


		//cout << "TrackReferenceKeyFrame ing : nmatches is " << nmatches << endl;

		if (nmatches < 15)
			return false;

		m_CurrentFrame.m_vpMapPoints = vpMapPointMatches;

		m_CurrentFrame.SetPose(m_LastFrame.m_Transformation);

		Optimizer::PoseOptimization(&m_CurrentFrame);

		int nmatchesMap = 0;
		for (int i = 0; i < m_CurrentFrame.m_nKeys; i++) {
			if (m_CurrentFrame.m_vpMapPoints[i]) {
				if (m_CurrentFrame.m_vbOutlier[i]) {
					MapPoint* pMP = m_CurrentFrame.m_vpMapPoints[i];

					m_CurrentFrame.m_vpMapPoints[i] = static_cast<MapPoint*>(NULL);
					m_CurrentFrame.m_vbOutlier[i] = false;
					 
					pMP->m_nLastFrameSeen = m_CurrentFrame.m_nFId;
					pMP->m_bTrackInView = false;
					
					nmatches--;
				} 
				else if (m_CurrentFrame.m_vpMapPoints[i]->Observations()>0)
					nmatchesMap++;
			}
		}

		//cout << "TrackReferenceKeyFrame ing : nmatchesMap is " << nmatchesMap << endl;
		 
		return nmatchesMap >= 10;
	}

	bool Tracking::TrackWithMotionModel() {
		 
		UpdateLastFrame();
		 
		m_CurrentFrame.SetPose(m_Velocity*m_LastFrame.m_Transformation);
		 
		fill(m_CurrentFrame.m_vpMapPoints.begin(), m_CurrentFrame.m_vpMapPoints.end(), static_cast<MapPoint*>(NULL));
		 
		ORBmatcher matcher(0.9, true);
		int th = 7;  
		int nmatches = matcher.SearchByProjection(m_CurrentFrame, m_LastFrame, th);
		 
		if (nmatches < 20) {
			fill(m_CurrentFrame.m_vpMapPoints.begin(), m_CurrentFrame.m_vpMapPoints.end(), static_cast<MapPoint*>(NULL));
			nmatches = matcher.SearchByProjection(m_CurrentFrame, m_LastFrame, 2 * th);
		}
		 
		if (nmatches < 20)
			return false;
		 
		Optimizer::PoseOptimization(&m_CurrentFrame);
		 
		int nmatchesMap = 0;
		for (int i = 0; i < m_CurrentFrame.m_nKeys; i++) {
			if (m_CurrentFrame.m_vpMapPoints[i]) {
				if (m_CurrentFrame.m_vbOutlier[i]) {
					MapPoint* pMP = m_CurrentFrame.m_vpMapPoints[i];

					m_CurrentFrame.m_vpMapPoints[i] = static_cast<MapPoint*>(NULL);
					m_CurrentFrame.m_vbOutlier[i] = false;
					 
					pMP->m_bTrackInView = false;
					pMP->m_nLastFrameSeen = m_CurrentFrame.m_nFId;

					nmatches--;
				}
		 
				else if (m_CurrentFrame.m_vpMapPoints[i]->Observations()>0)
					nmatchesMap++;
			}
		}
		return nmatchesMap >= 10;
	}

	void Tracking::UpdateLastFrame() {

		// Update pose according to reference keyframe 
		KeyFrame* pRef = m_LastFrame.m_pReferenceKF;
		cv::Mat Tlr = m_lRelativeFramePoses.back();

		m_LastFrame.SetPose(Tlr*pRef->GetPose());
		 
		if (m_nLastKeyFrameId == m_LastFrame.m_nFId)
			return;
		 
		vector<pair<float, int> > vDepthIdx;
		vDepthIdx.reserve(m_LastFrame.m_nKeys);
		for (int i = 0; i < m_LastFrame.m_nKeys; i++) {
			float z = m_LastFrame.m_vfDepth[i];
			if (z > 0) {
				vDepthIdx.push_back(make_pair(z, i));
			}
		}

		if (vDepthIdx.empty())
			return;
		 
		sort(vDepthIdx.begin(), vDepthIdx.end());
 
		int nPoints = 0; 
		for (size_t j = 0; j < vDepthIdx.size(); j++) {

			int i = vDepthIdx[j].second;
			 
			bool bCreateNew = false;

			MapPoint* pMP = m_LastFrame.m_vpMapPoints[i];
			if (!pMP)
				bCreateNew = true;
			else if (pMP->Observations() < 1) {
				bCreateNew = true;
			}

			if (bCreateNew) {
				cv::Mat x3D = m_LastFrame.ComputeWorldPos(i);
				MapPoint* pNewMP = new MapPoint(x3D, &m_LastFrame, i, m_pMap);

				m_LastFrame.m_vpMapPoints[i] = pNewMP;
				
				m_lpTemporalPoints.push_back(pNewMP);
				nPoints++;
			}
			else {
				nPoints++;
			}
			 
			if (vDepthIdx[j].first > m_fThDepth || nPoints > 100)
				break;
		}
	}

	bool Tracking::Relocalization() {
		 
		m_CurrentFrame.ComputeBoW();
		 
		vector<KeyFrame*> vpCandidateKFs = m_pKeyFrameDB->DetectRelocalizationCandidates(&m_CurrentFrame);

		if (vpCandidateKFs.empty())
			return false;

		const int nKFs = vpCandidateKFs.size();
		cout << "vpCandidateKFs.size() " << nKFs << endl;

		ORBmatcher matcher(0.75, true);
		 
		vector<PnPsolver*> vpPnPsolvers;
		vpPnPsolvers.resize(nKFs);
		 
		vector<vector<MapPoint*> > vvpMapPointMatches; 
		vvpMapPointMatches.resize(nKFs);
		 
		vector<bool> vbDiscarded; 
		vbDiscarded.resize(nKFs);

		int nCandidates = 0;

		for (int i = 0; i < nKFs; i++) {

			KeyFrame* pKF = vpCandidateKFs[i];
			if (pKF->isBad())
				vbDiscarded[i] = true;
			else { 
				int nmatches = matcher.SearchByBoW(pKF, m_CurrentFrame, vvpMapPointMatches[i]);
				 
				if (nmatches < 15) {
					vbDiscarded[i] = true;
					continue;
				} else {
					PnPsolver* pSolver = new PnPsolver(m_CurrentFrame, vvpMapPointMatches[i]);
					//pSolver->setRansacParameters();
					pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);
					vpPnPsolvers[i] = pSolver;
					nCandidates++;
				}
			}
		}
		 
		bool bMatch = false; 
		ORBmatcher matcher2(0.9, true);

		while (nCandidates > 0 && !bMatch)
		{
			for (int i = 0; i < nKFs; i++)
			{
				if (vbDiscarded[i])
					continue;

				// Perform 5 Ransac Iterations
				vector<bool> vbInliers;
				int nInliers;
				bool bNoMore;

				PnPsolver* pSolver = vpPnPsolvers[i];
				cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

				// If Ransac reachs max. iterations discard keyframe
				if (bNoMore)
				{
					vbDiscarded[i] = true;
					nCandidates--;
				}

				// If a Camera Pose is computed, optimize
				if (!Tcw.empty())
				{
					Tcw.copyTo(m_CurrentFrame.m_Transformation);

					set<MapPoint*> sFound;

					const int np = vbInliers.size();

					for (int j = 0; j < np; j++)
					{
						if (vbInliers[j])
						{
							m_CurrentFrame.m_vpMapPoints[j] = vvpMapPointMatches[i][j];
							sFound.insert(vvpMapPointMatches[i][j]);
						}
						else
							m_CurrentFrame.m_vpMapPoints[j] = NULL;
					}

					int nGood = Optimizer::PoseOptimization(&m_CurrentFrame);

					if (nGood < 10)
						continue;

					for (int io = 0; io < m_CurrentFrame.m_nKeys; io++)
						if (m_CurrentFrame.m_vbOutlier[io])
							m_CurrentFrame.m_vpMapPoints[io] = static_cast<MapPoint*>(NULL);

					// If few inliers, search by projection in a coarse window and optimize again
					if (nGood < 50)
					{
						int nadditional = matcher2.SearchByProjection(m_CurrentFrame, vpCandidateKFs[i], sFound, 10, 100);

						if (nadditional + nGood >= 50)
						{
							nGood = Optimizer::PoseOptimization(&m_CurrentFrame);

							// If many inliers but still not enough, search by projection again in a narrower window
							// the camera has been already optimized with many points
							if (nGood > 30 && nGood < 50)
							{
								sFound.clear();
								for (int ip = 0; ip < m_CurrentFrame.m_nKeys; ip++)
									if (m_CurrentFrame.m_vpMapPoints[ip])
										sFound.insert(m_CurrentFrame.m_vpMapPoints[ip]);
								nadditional = matcher2.SearchByProjection(m_CurrentFrame, vpCandidateKFs[i], sFound, 3, 64);

								// Final optimization
								if (nGood + nadditional >= 50)
								{
									nGood = Optimizer::PoseOptimization(&m_CurrentFrame);

									for (int io = 0; io < m_CurrentFrame.m_nKeys; io++)
										if (m_CurrentFrame.m_vbOutlier[io])
											m_CurrentFrame.m_vpMapPoints[io] = NULL;
								}
							}
						}
					}


					// If the pose is supported by enough inliers stop ransacs and continue
					if (nGood >= 50)
					{
						bMatch = true;
						break;
					}
				}
			}
		}

		if (!bMatch) {
			cout << "no m_nLastRelocFrameId" << m_nLastRelocFrameId << endl;
			return false;
		} else {
			m_nLastRelocFrameId = m_CurrentFrame.m_nFId;
			cout << m_CurrentFrame.m_Transformation << endl;
			cout << "m_nLastRelocFrameId" << m_nLastRelocFrameId << endl;
			return true;
		}
	}
	
	bool Tracking::TrackLocalMap() {
		// We have an estimation of the camera pose and some map points tracked in the frame.
		// We retrieve the local map and try to find matches to points in the local map. 
		UpdateLocalMap();
		SearchLocalPoints();
		 
		Optimizer::PoseOptimization(&m_CurrentFrame);
		 
		m_nMatchesInliers = 0;
		 
		for (int i = 0; i < m_CurrentFrame.m_nKeys; i++) { 
			if (m_CurrentFrame.m_vpMapPoints[i]) {
				if (!m_CurrentFrame.m_vbOutlier[i]) {
					m_CurrentFrame.m_vpMapPoints[i]->IncreaseFound();  
					if (m_CurrentFrame.m_vpMapPoints[i]->Observations()>0)
						m_nMatchesInliers++;
				}
			}
		}
		 
		if (m_CurrentFrame.m_nFId < m_nLastRelocFrameId + m_MaxFrames && m_nMatchesInliers < 50)
			return false;

		if (m_nMatchesInliers < 30)
			return false;
		else
			return true;

	}

	void Tracking::UpdateLocalMap() {
		 
		UpdateLocalKeyFrames();
		UpdateLocalMapPoints();
	}

	void Tracking::UpdateLocalKeyFrames() {
		  
		map<KeyFrame*, int> keyframeCounter;
		for (int i = 0; i < m_CurrentFrame.m_nKeys; i++) {

			if (m_CurrentFrame.m_vpMapPoints[i]) {

				MapPoint* pMP = m_CurrentFrame.m_vpMapPoints[i];
				if (!pMP->isBad()) {
					const map<KeyFrame*, size_t> observations = pMP->GetObservations();
					for (map<KeyFrame*, size_t>::const_iterator it = observations.begin(), itend = observations.end(); it != itend; it++)
						keyframeCounter[it->first]++;
				} else {
					m_CurrentFrame.m_vpMapPoints[i] = NULL;
				}
			}
		}

		if (keyframeCounter.empty())
			return;

		int max = 0; 
		KeyFrame* pKFmax = static_cast<KeyFrame*>(NULL); 

		m_vpLocalKeyFrames.clear();
		m_vpLocalKeyFrames.reserve(3 * keyframeCounter.size());
		 
		for (map<KeyFrame*, int>::const_iterator it = keyframeCounter.begin(), itEnd = keyframeCounter.end(); it != itEnd; it++) {
			KeyFrame* pKF = it->first;

			if (pKF->isBad())
				continue;

			if (it->second > max) {
				max = it->second;
				pKFmax = pKF;
			}

			m_vpLocalKeyFrames.push_back(it->first);
			pKF->m_nTrackReferenceForFrame = m_CurrentFrame.m_nFId;
		}
		 
		if (pKFmax) {
			m_pReferenceKF = pKFmax;
			m_CurrentFrame.m_pReferenceKF = m_pReferenceKF; 
		}

		vector<KeyFrame*> vpLocalKeyFrames = m_vpLocalKeyFrames;
		 
		for (vector<KeyFrame*>::const_iterator itKF = m_vpLocalKeyFrames.begin(), itEndKF = m_vpLocalKeyFrames.end(); itKF != itEndKF; itKF++) {
			  
			if (vpLocalKeyFrames.size() > 80)
				break;

			KeyFrame* pKF = *itKF;
			 
			const vector<KeyFrame*> vNeighs = m_pCoGraph->GetBestCovisibilityKeyFrames(pKF, 10);

			for (vector<KeyFrame*>::const_iterator itNeighKF = vNeighs.begin(), itEndNeighKF = vNeighs.end(); itNeighKF != itEndNeighKF; itNeighKF++) {

				KeyFrame* pNeighKF = *itNeighKF;
				if (!pNeighKF->isBad()) {
					if (pNeighKF->m_nTrackReferenceForFrame != m_CurrentFrame.m_nFId) {
						// m_vpLocalKeyFrames.push_back(pNeighKF);
						vpLocalKeyFrames.push_back(pNeighKF);
						pNeighKF->m_nTrackReferenceForFrame = m_CurrentFrame.m_nFId;
						break;
					}
				}
			}
			 
			const set<KeyFrame*> spChilds = m_pSpanTree->GetChilds(pKF);
			for (set<KeyFrame*>::const_iterator sit = spChilds.begin(), send = spChilds.end(); sit != send; sit++) {
				KeyFrame* pChildKF = *sit;
				if (!pChildKF->isBad()) {
					if (pChildKF->m_nTrackReferenceForFrame != m_CurrentFrame.m_nFId) {
						// m_vpLocalKeyFrames.push_back(pChildKF);
						vpLocalKeyFrames.push_back(pChildKF);
						pChildKF->m_nTrackReferenceForFrame = m_CurrentFrame.m_nFId;
						break;
					}
				}
			}

			KeyFrame* pParent = m_pSpanTree->GetParent(pKF);
			if (pParent) {
				if (pParent->m_nTrackReferenceForFrame != m_CurrentFrame.m_nFId) {
					// m_vpLocalKeyFrames.push_back(pParent);
					vpLocalKeyFrames.push_back(pParent);
					pParent->m_nTrackReferenceForFrame = m_CurrentFrame.m_nFId;
					break;
				}
			}
		}

		m_vpLocalKeyFrames = vpLocalKeyFrames;

	}

	void Tracking::UpdateLocalMapPoints() {
		m_vpLocalMapPoints.clear();
		 
		for (vector<KeyFrame*>::const_iterator itKF = m_vpLocalKeyFrames.begin(), itEndKF = m_vpLocalKeyFrames.end(); itKF != itEndKF; itKF++) {
			KeyFrame* pKF = *itKF;
			const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

			for (vector<MapPoint*>::const_iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end(); itMP != itEndMP; itMP++) {
				MapPoint* pMP = *itMP;
				if (!pMP)
					continue;
				if (pMP->m_nTrackReferenceForFrame == m_CurrentFrame.m_nFId)
					continue;
				if (!pMP->isBad()) {
					m_vpLocalMapPoints.push_back(pMP);
					pMP->m_nTrackReferenceForFrame = m_CurrentFrame.m_nFId;
				}
			}
		}
	}

	void Tracking::SearchLocalPoints() {
		 
		for (vector<MapPoint*>::iterator vit = m_CurrentFrame.m_vpMapPoints.begin(), vend = m_CurrentFrame.m_vpMapPoints.end(); vit != vend; vit++) {

			MapPoint* pMP = *vit;
			if (pMP) {
				if (pMP->isBad()) {
					*vit = static_cast<MapPoint*>(NULL);
				} 
				else {
					pMP->IncreaseVisible();
					pMP->m_nLastFrameSeen = m_CurrentFrame.m_nFId;
					pMP->m_bTrackInView = false;
				}
			}
		}

		int nToMatch = 0; 

		for (vector<MapPoint*>::iterator vit = m_vpLocalMapPoints.begin(), vend = m_vpLocalMapPoints.end(); vit != vend; vit++) {
			MapPoint* pMP = *vit; 
			if (pMP->m_nLastFrameSeen == m_CurrentFrame.m_nFId)
				continue;
			if (pMP->isBad())
				continue;
			 
			if (m_CurrentFrame.isInFrustum(pMP, 0.5)) {
				pMP->IncreaseVisible();  
				nToMatch++;
			}
		}
		 
		if (nToMatch > 0) {
			ORBmatcher matcher(0.8);
			int th = 3;
			 
			if (m_CurrentFrame.m_nFId < m_nLastRelocFrameId + 2)
				th = 5;
			matcher.SearchByProjection(m_CurrentFrame, m_vpLocalMapPoints, th);
		}

	}

	bool Tracking::NeedNewKeyFrame() {
		 
		if (m_pLocalMapper->isStopped() || m_pLocalMapper->stopRequested())
			return false;
		 
		const int nKFs = m_pMap->KeyFramesInMap();
		 
		if (m_CurrentFrame.m_nFId < m_nLastRelocFrameId + m_MaxFrames && nKFs > m_MaxFrames)
			return false;
		 
		int nMinObs = 3;
		if (nKFs <= 2)
			nMinObs = 2;
		int nRefMatches = m_pReferenceKF->TrackedMapPoints(nMinObs); 
		 
		bool bLocalMappingIdle = m_pLocalMapper->AcceptKeyFrames();
		 
		int nMap = 0;
		int nTotal = 0;

		for (int i = 0; i < m_CurrentFrame.m_nKeys; i++) {
			if (m_CurrentFrame.m_vfDepth[i] > 0 && m_CurrentFrame.m_vfDepth[i] < m_fThDepth) {
				nTotal++;
				if (m_CurrentFrame.m_vpMapPoints[i])
					if (m_CurrentFrame.m_vpMapPoints[i]->Observations() > 0)
						nMap++;
			}
		}
 
		const float ratioMap = (float)nMap / fmax(1.0f, nTotal);
		 
		float thRefRatio = 0.75f;
		if (nKFs < 2)
			thRefRatio = 0.4f;

		float thMapRatio = 0.35f;
		if (m_nMatchesInliers > 300)
			thMapRatio = 0.20f;
		 
		const bool c1a = m_CurrentFrame.m_nFId >= m_nLastKeyFrameId + m_MaxFrames; 
		const bool c1b = (m_CurrentFrame.m_nFId >= m_nLastKeyFrameId + m_MinFrames && bLocalMappingIdle); 
		const bool c1c = (m_nMatchesInliers < nRefMatches*0.25 || ratioMap < 0.3f);
		 
		const bool c2 = ((m_nMatchesInliers < nRefMatches*thRefRatio || ratioMap < thMapRatio) && m_nMatchesInliers > 15);

		if ((c1a || c1b || c1c) && c2) {
			// If the mapping accepts keyframes, insert keyframe.
			// Otherwise send a signal to interrupt BA 
			if (bLocalMappingIdle) {
				return true;
			}
			else {
				m_pLocalMapper->InterruptBA();
				 
				if (m_pLocalMapper->KeyframesInQueue() < 3)
					return true;
				else
					return false;
			}
		}
		else
			return false;  


	}

	void Tracking::CreateNewKeyFrame() {
		 
		if (!m_pLocalMapper->SetNotStop(true))
			return;

		KeyFrame* pKF = new KeyFrame(m_CurrentFrame); 
		cout << "KeyFrame's Frame Id" << pKF->m_nFId << endl; 
		 
		m_pReferenceKF = pKF;
		m_CurrentFrame.m_pReferenceKF = pKF;  
		 
		m_CurrentFrame.UpdatePoseMatrices();
		 
		vector<pair<float, int> > vDepthIdx;
		vDepthIdx.reserve(m_CurrentFrame.m_nKeys);
		for (int i = 0; i < m_CurrentFrame.m_nKeys; i++) {
			float z = m_CurrentFrame.m_vfDepth[i];
			if (z > 0)
				vDepthIdx.push_back(make_pair(z, i));
		}
		 
		cout << "Before adding MapPoint, the number is " << m_pMap->GetAllMapPoints().size() << endl;
		 
		if (!vDepthIdx.empty()) {
			sort(vDepthIdx.begin(), vDepthIdx.end());

			//cout << "Add the new MapPoint based on the depth map" << endl;

			int nPoints = 0;
			for (size_t j = 0; j < vDepthIdx.size(); j++) {

				int i = vDepthIdx[j].second;

				bool bCreateNew = false;

				MapPoint* pMP = m_CurrentFrame.m_vpMapPoints[i];

				if (!pMP)
					bCreateNew = true;
				else if (pMP->Observations() < 1) { 
					bCreateNew = true;
					m_CurrentFrame.m_vpMapPoints[i] = static_cast<MapPoint*>(NULL);
				}

				if (bCreateNew) { 
					cv::Mat x3D = m_CurrentFrame.ComputeWorldPos(i);
					MapPoint* pNewMP = new MapPoint(x3D, pKF, m_pMap);
					pNewMP->AddObservation(pKF, i);

					pKF->AddMapPoint(pNewMP, i);

					pNewMP->ComputeDistinctiveDescriptors();
					pNewMP->UpdateNormalAndDepth();

					m_pMap->AddMapPoint(pNewMP);

					m_CurrentFrame.m_vpMapPoints[i] = pNewMP;
					nPoints++;
				}
				else {
					nPoints++;
				}
				 
				if (vDepthIdx[j].first > m_fThDepth || nPoints > 100)
					break;
			}
		}
		

		m_pLocalMapper->InsertKeyFrame(pKF);
		cout << "After adding MapPoint, the number is " << m_pMap->GetAllMapPoints().size() << endl; 
		m_pLocalMapper->SetNotStop(false);

		m_nLastKeyFrameId = m_CurrentFrame.m_nFId;
		m_pLastKeyFrame = pKF;
	}




	void Tracking::Reset() {

		//cout << "System Reseting" << endl;
		// Reset Local Mapping
		cout << "Reseting Local Mapper...";
		m_pLocalMapper->RequestReset();
		cout << " done" << endl;

		// Reset Loop Closing
		cout << "Reseting Loop Closing...";
		m_pLoopCloser->RequestReset();
		cout << " done" << endl;
	}

} // namespace SLAMRecon

