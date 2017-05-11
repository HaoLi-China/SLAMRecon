/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
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

#include "Optimizer.h"
#include "Converter.h"

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <g2o/types/sba/types_six_dof_expmap.h>

//#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include "types_seven_dof_expmap.h"

#include <g2o/core/robust_kernel_impl.h>


namespace SLAMRecon {

	int Optimizer::PoseOptimization(Frame* pFrame){

		g2o::SparseOptimizer optimizer;

		g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
		linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
		
		g2o::BlockSolver_6_3 * blockSolver = new g2o::BlockSolver_6_3(linearSolver);

		g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);

		optimizer.setAlgorithm(solver);

		// Set Frame vertex
		g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate(Converter::toSE3Quat(pFrame->m_Transformation));
		vSE3->setId(0);
		vSE3->setFixed(false); 
		optimizer.addVertex(vSE3);

		// Set MapPoint vertices
		const int N = pFrame->m_nKeys;

		vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
		vector<size_t> vnIndexEdgeMono;
		vpEdgesMono.reserve(N);
		vnIndexEdgeMono.reserve(N);

		vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
		vector<size_t> vnIndexEdgeStereo;
		vpEdgesStereo.reserve(N);
		vnIndexEdgeStereo.reserve(N);

		const float deltaMono = sqrt(5.991);
		const float deltaStereo = sqrt(7.815);


		int nInitialCorrespondences = 0;

		{
			unique_lock<mutex> lock(MapPoint::m_GlobalMutex);

			for (int i = 0; i < N; i++) {
				MapPoint* pMP = pFrame->m_vpMapPoints[i];
				if (pMP) {

					if (pFrame->m_vuRight[i] < 0) {
						nInitialCorrespondences++;
						pFrame->m_vbOutlier[i] = false;

						g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();

						Eigen::Matrix<double, 2, 1> obs;
						const cv::KeyPoint &kpUn = pFrame->m_vKeysUn[i];
						obs << kpUn.pt.x, kpUn.pt.y;
						e->setMeasurement(obs);

						e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));

						const float invSigma2 = pFrame->m_pPLevelInfo->m_vInvLevelSigma2[kpUn.octave];
						e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
	
						g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
						rk->setDelta(deltaMono); 
						e->setRobustKernel(rk);

						e->fx = pFrame->m_pCameraInfo->m_fx;
						e->fy = pFrame->m_pCameraInfo->m_fy;
						e->cx = pFrame->m_pCameraInfo->m_cx;
						e->cy = pFrame->m_pCameraInfo->m_cy;
						cv::Mat Xw = pMP->GetWorldPos();
						e->Xw[0] = Xw.at<float>(0);
						e->Xw[1] = Xw.at<float>(1);
						e->Xw[2] = Xw.at<float>(2);


						optimizer.addEdge(e);

						vpEdgesMono.push_back(e);
						vnIndexEdgeMono.push_back(i);
					}
					else {
						nInitialCorrespondences++;
						pFrame->m_vbOutlier[i] = false;

						g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

						Eigen::Matrix<double, 3, 1> obs;
						const cv::KeyPoint &kpUn = pFrame->m_vKeysUn[i];
						const float &kp_ur = pFrame->m_vuRight[i];
						obs << kpUn.pt.x, kpUn.pt.y, kp_ur;
						e->setMeasurement(obs);

						e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));

						const float invSigma2 = pFrame->m_pPLevelInfo->m_vInvLevelSigma2[kpUn.octave];
						e->setInformation(Eigen::Matrix3d::Identity()*invSigma2);

						
						g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
						rk->setDelta(deltaStereo); 
						e->setRobustKernel(rk);

						e->fx = pFrame->m_pCameraInfo->m_fx;
						e->fy = pFrame->m_pCameraInfo->m_fy;
						e->cx = pFrame->m_pCameraInfo->m_cx;
						e->cy = pFrame->m_pCameraInfo->m_cy;
						e->bf = pFrame->m_pCameraInfo->m_bf;
						cv::Mat Xw = pMP->GetWorldPos();
						e->Xw[0] = Xw.at<float>(0);
						e->Xw[1] = Xw.at<float>(1);
						e->Xw[2] = Xw.at<float>(2);

						optimizer.addEdge(e);

						vpEdgesStereo.push_back(e);
						vnIndexEdgeStereo.push_back(i);
					}
				}
			}
		}

		if (nInitialCorrespondences < 3)
			return 0;

		const float chi2Mono[4] = { 5.991, 5.991, 5.991, 5.991 };
		const float chi2Stereo[4] = { 7.815, 7.815, 7.815, 7.815 };
		const int its[4] = { 10, 10, 10, 10 };


		int nBad = 0;
		for (size_t it = 0; it < 4; it++) {

			vSE3->setEstimate(Converter::toSE3Quat(pFrame->m_Transformation));
			optimizer.initializeOptimization(0); 
			optimizer.optimize(its[it]); 

			nBad = 0;
			for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {

				g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

				const size_t idx = vnIndexEdgeMono[i];

				if (pFrame->m_vbOutlier[idx]) {
					e->computeError();
				}

				const float chi2 = e->chi2();

				if (chi2 > chi2Mono[it]) {
					pFrame->m_vbOutlier[idx] = true;
					e->setLevel(1);
					nBad++;
				}
				else {
					pFrame->m_vbOutlier[idx] = false;
					e->setLevel(0);
				}

				if (it == 2)
					e->setRobustKernel(0);
			}

			for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
				g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

				const size_t idx = vnIndexEdgeStereo[i];

				if (pFrame->m_vbOutlier[idx]) {
					e->computeError();
				}

				const float chi2 = e->chi2();

				if (chi2 > chi2Stereo[it]) {
					pFrame->m_vbOutlier[idx] = true;
					e->setLevel(1);
					nBad++;
				}
				else {
					e->setLevel(0);
					pFrame->m_vbOutlier[idx] = false;
				}

				if (it == 2)
					e->setRobustKernel(0);
			}

			if (optimizer.edges().size() < 10)
				break;
		}

		g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
		g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
		cv::Mat pose = Converter::toCvMat(SE3quat_recov);
		pFrame->SetPose(pose);

		return nInitialCorrespondences - nBad;
	}

	void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool* pbStopFlag, Map* pMap, CovisibilityGraph* pCoGraph) {

		list<KeyFrame*> lLocalKeyFrames;

		lLocalKeyFrames.push_back(pKF);
		pKF->m_nBALocalForKF = pKF->m_nKFId;

		const vector<KeyFrame*> vNeighKFs = pCoGraph->GetVectorCovisibleKeyFrames(pKF);

		for (int i = 0, iend = vNeighKFs.size(); i < iend; i++) {
			KeyFrame* pKFi = vNeighKFs[i];
			pKFi->m_nBALocalForKF = pKF->m_nKFId;
			if (!pKFi->isBad())
				lLocalKeyFrames.push_back(pKFi);
		}

		list<MapPoint*> lLocalMapPoints;
		for (list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++) {

			vector<MapPoint*> vpMPs = (*lit)->GetMapPointMatches();
			for (vector<MapPoint*>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++) {
				MapPoint* pMP = *vit;
				if (pMP)
					if (!pMP->isBad())
						if (pMP->m_nBALocalForKF != pKF->m_nKFId) {
							lLocalMapPoints.push_back(pMP);
							pMP->m_nBALocalForKF = pKF->m_nKFId;
						}
			}
		}

		list<KeyFrame*> lFixedKeyFrames;
		for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
			map<KeyFrame*, size_t> observations = (*lit)->GetObservations();
			for (map<KeyFrame*, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
				KeyFrame* pKFi = mit->first;

				if (pKFi->m_nBALocalForKF != pKF->m_nKFId && pKFi->m_nBAFixedForKF != pKF->m_nKFId) {
					pKFi->m_nBALocalForKF = pKF->m_nKFId;
					if (!pKFi->isBad())
						lFixedKeyFrames.push_back(pKFi);
				}
			}
		}

		g2o::SparseOptimizer optimizer;

		g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
		linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

		g2o::BlockSolver_6_3 * blockSolver = new g2o::BlockSolver_6_3(linearSolver);

		g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);

		optimizer.setAlgorithm(solver);

		if (pbStopFlag)
			optimizer.setForceStopFlag(pbStopFlag);

		unsigned long maxKFid = 0; 

		for (list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++) {
			KeyFrame* pKFi = *lit;
			g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
			vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
			vSE3->setId(pKFi->m_nKFId); 
			vSE3->setFixed(pKFi->m_nKFId == 0);
			optimizer.addVertex(vSE3);

			if (pKFi->m_nKFId > maxKFid)
				maxKFid = pKFi->m_nKFId;
		}

		for (list<KeyFrame*>::iterator lit = lFixedKeyFrames.begin(), lend = lFixedKeyFrames.end(); lit != lend; lit++) {
			KeyFrame* pKFi = *lit;
			g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
			vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
			vSE3->setId(pKFi->m_nKFId);
			vSE3->setFixed(true);
			optimizer.addVertex(vSE3);

			if (pKFi->m_nKFId > maxKFid)
				maxKFid = pKFi->m_nKFId;
		}

		const int nExpectedSize = (lLocalKeyFrames.size() + lFixedKeyFrames.size())*lLocalMapPoints.size();

		vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
		vector<KeyFrame*> vpKFVertexesMono;
		vector<MapPoint*> vpMapPointVertexesMono;
		vpEdgesMono.reserve(nExpectedSize);
		vpKFVertexesMono.reserve(nExpectedSize);
		vpMapPointVertexesMono.reserve(nExpectedSize);

		vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
		vector<KeyFrame*> vpKFVertexesStereo;
		vector<MapPoint*> vpMapPointVertexesStereo;
		vpEdgesStereo.reserve(nExpectedSize);
		vpKFVertexesStereo.reserve(nExpectedSize);
		vpMapPointVertexesStereo.reserve(nExpectedSize);

		const float thHuberMono = sqrt(5.991);
		const float thHuberStereo = sqrt(7.815);

		for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {

			MapPoint* pMP = *lit;
			g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
			int id = pMP->m_nMPId + maxKFid + 1;
			vPoint->setId(id);
			vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
			vPoint->setMarginalized(true); 
			optimizer.addVertex(vPoint);

			const map<KeyFrame*, size_t> observations = pMP->GetObservations();

			for (map<KeyFrame*, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
				KeyFrame* pKFi = mit->first;

				if (!pKFi->isBad()) {


					if (pKFi->m_vuRight[mit->second] < 0) {

						g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

						const cv::KeyPoint &kpUn = pKFi->m_vKeysUn[mit->second];
						Eigen::Matrix<double, 2, 1> obs;
						obs << kpUn.pt.x, kpUn.pt.y;
						e->setMeasurement(obs);


						e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
						e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->m_nKFId)));


						const float &invSigma2 = pKFi->m_pPLevelInfo->m_vInvLevelSigma2[kpUn.octave];
						e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

						g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
						rk->setDelta(thHuberMono);
						e->setRobustKernel(rk);


						e->fx = pKFi->m_pCameraInfo->m_fx;
						e->fy = pKFi->m_pCameraInfo->m_fy;
						e->cx = pKFi->m_pCameraInfo->m_cx;
						e->cy = pKFi->m_pCameraInfo->m_cy;

						optimizer.addEdge(e);
						vpEdgesMono.push_back(e);
						vpKFVertexesMono.push_back(pKFi);
						vpMapPointVertexesMono.push_back(pMP);
					}
					else {
						g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

						const cv::KeyPoint &kpUn = pKFi->m_vKeysUn[mit->second];
						const float kp_ur = pKFi->m_vuRight[mit->second];
						Eigen::Matrix<double, 3, 1> obs;
						obs << kpUn.pt.x, kpUn.pt.y, kp_ur;
						e->setMeasurement(obs);


						e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
						e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->m_nKFId)));


						const float &invSigma2 = pKFi->m_pPLevelInfo->m_vInvLevelSigma2[kpUn.octave];
						e->setInformation(Eigen::Matrix3d::Identity()*invSigma2);

						g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
						rk->setDelta(thHuberStereo);
						e->setRobustKernel(rk);

						e->fx = pKFi->m_pCameraInfo->m_fx;
						e->fy = pKFi->m_pCameraInfo->m_fy;
						e->cx = pKFi->m_pCameraInfo->m_cx;
						e->cy = pKFi->m_pCameraInfo->m_cy;
						e->bf = pKFi->m_pCameraInfo->m_bf;

						optimizer.addEdge(e);
						vpEdgesStereo.push_back(e);
						vpKFVertexesStereo.push_back(pKFi);
						vpMapPointVertexesStereo.push_back(pMP);
					}
				}
			}
		}

		if (pbStopFlag)
			if (*pbStopFlag)
				return;

		optimizer.initializeOptimization(0);
		optimizer.optimize(5);

		bool bDoMore = true;

		if (pbStopFlag)
			if (*pbStopFlag)
				bDoMore = false;

		if (bDoMore) {

			for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
				g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
				MapPoint* pMP = vpMapPointVertexesMono[i];

				if (pMP->isBad())
					continue;

				if (e->chi2() > thHuberMono || !e->isDepthPositive()) {
					e->setLevel(1);
				}
				e->setRobustKernel(0);
			}

			for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
				g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
				MapPoint* pMP = vpMapPointVertexesStereo[i];

				if (pMP->isBad())
					continue;

				if (e->chi2() > thHuberStereo || !e->isDepthPositive()) {
					e->setLevel(1);
				}

				e->setRobustKernel(0);
			}

			optimizer.initializeOptimization(0);
			optimizer.optimize(10);
		}

		vector<pair<KeyFrame*, MapPoint*> > vToErase;
		vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

		for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
			g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
			MapPoint* pMP = vpMapPointVertexesMono[i];

			if (pMP->isBad())
				continue;

			if (e->chi2() > thHuberMono || !e->isDepthPositive()) {
				KeyFrame* pKFi = vpKFVertexesMono[i];
				vToErase.push_back(make_pair(pKFi, pMP));
			}
		}

		for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
			g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
			MapPoint* pMP = vpMapPointVertexesStereo[i];

			if (pMP->isBad())
				continue;

			if (e->chi2() > thHuberStereo || !e->isDepthPositive())
			{
				KeyFrame* pKFi = vpKFVertexesStereo[i];
				vToErase.push_back(make_pair(pKFi, pMP));
			}
		}

		// Get Map Mutex
		unique_lock<mutex> lock(pMap->m_MutexMapUpdate);

		if (!vToErase.empty()) {
			for (size_t i = 0; i < vToErase.size(); i++) {
				KeyFrame* pKFi = vToErase[i].first;
				MapPoint* pMPi = vToErase[i].second;
				pKFi->EraseMapPoint(pMPi);
				pMPi->EraseObservation(pKFi);
			}
		}

		vector<KeyFrame*> vpKFs(lLocalKeyFrames.begin(), lLocalKeyFrames.end());
		sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

		// update KeyFrame CameraPose
		for (list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++) {
			KeyFrame* pKF = *lit;
			g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->m_nKFId));
			g2o::SE3Quat SE3quat = vSE3->estimate();
			pKF->SetPose(Converter::toCvMat(SE3quat));
			
			pMap->addModifiedKeyFrame(pKF);

		}

		// update MapPoint location
		for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
			MapPoint* pMP = *lit;
			g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->m_nMPId + maxKFid + 1));
			pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
			pMP->UpdateNormalAndDepth();
		}

	}

	int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches12, g2o::Sim3 &g2oS12, const float th2) {
		 
		g2o::SparseOptimizer optimizer;
		 
		g2o::BlockSolverX::LinearSolverType * linearSolver;

		linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

		g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

		g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
		optimizer.setAlgorithm(solver);
		 
		const cv::Mat &K1 = pKF1->m_pCameraInfo->m_K;
		const cv::Mat &K2 = pKF2->m_pCameraInfo->m_K;

		 
		const cv::Mat R1w = pKF1->GetRotation();
		const cv::Mat t1w = pKF1->GetTranslation();
		const cv::Mat R2w = pKF2->GetRotation();
		const cv::Mat t2w = pKF2->GetTranslation();
		 
		g2o::VertexSim3Expmap *vSim3 = new g2o::VertexSim3Expmap();
		vSim3->_fix_scale = true;
		vSim3->setEstimate(g2oS12);
		vSim3->setId(0);
		vSim3->setFixed(false);
		vSim3->_principle_point1[0] = K1.at<float>(0, 2);
		vSim3->_principle_point1[1] = K1.at<float>(1, 2);
		vSim3->_focal_length1[0] = K1.at<float>(0, 0);
		vSim3->_focal_length1[1] = K1.at<float>(1, 1);
		vSim3->_principle_point2[0] = K2.at<float>(0, 2);
		vSim3->_principle_point2[1] = K2.at<float>(1, 2);
		vSim3->_focal_length2[0] = K2.at<float>(0, 0);
		vSim3->_focal_length2[1] = K2.at<float>(1, 1);
		optimizer.addVertex(vSim3);

		// Set MapPoint vertices
		const int N = vpMatches12.size();
		const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();

		vector<g2o::EdgeSim3ProjectXYZ*> vpEdges12;
		vector<g2o::EdgeInverseSim3ProjectXYZ*> vpEdges21;
		vector<size_t> vnIndexEdge;

		vnIndexEdge.reserve(2 * N);
		vpEdges12.reserve(2 * N);
		vpEdges21.reserve(2 * N);

		const float deltaHuber = sqrt(th2);

		int nCorrespondences = 0;

		for (int i = 0; i < N; i++) {
			 
			if (!vpMatches12[i])
				continue;
			 
			MapPoint* pMP1 = vpMapPoints1[i];
			MapPoint* pMP2 = vpMatches12[i];

			const int id1 = 2 * i + 1;
			const int id2 = 2 * (i + 1);

			const int i2 = pMP2->GetIndexInKeyFrame(pKF2);

			if (pMP1 && pMP2) {

				if (!pMP1->isBad() && !pMP2->isBad() && i2 >= 0) {
					g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
					cv::Mat P3D1w = pMP1->GetWorldPos();
					cv::Mat P3D1c = R1w*P3D1w + t1w;
					vPoint1->setEstimate(Converter::toVector3d(P3D1c));
					vPoint1->setId(id1);
					vPoint1->setFixed(true);
					optimizer.addVertex(vPoint1);

					g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
					cv::Mat P3D2w = pMP2->GetWorldPos();
					cv::Mat P3D2c = R2w*P3D2w + t2w;
					vPoint2->setEstimate(Converter::toVector3d(P3D2c));
					vPoint2->setId(id2);
					vPoint2->setFixed(true);
					optimizer.addVertex(vPoint2);
				}
				else
					continue;
			}
			else
				continue;

			nCorrespondences++;

			Eigen::Matrix<double, 2, 1> obs1;
			const cv::KeyPoint &kpUn1 = pKF1->m_vKeysUn[i];
			obs1 << kpUn1.pt.x, kpUn1.pt.y;

			g2o::EdgeSim3ProjectXYZ* e12 = new g2o::EdgeSim3ProjectXYZ();
			e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id2)));
			e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
			e12->setMeasurement(obs1);
			const float &invSigmaSquare1 = pKF1->m_pPLevelInfo->m_vInvLevelSigma2[kpUn1.octave];
			e12->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare1);

			g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
			e12->setRobustKernel(rk1);
			rk1->setDelta(deltaHuber);
			optimizer.addEdge(e12);

			// Set edge x2 = S21*X1
			Eigen::Matrix<double, 2, 1> obs2;
			const cv::KeyPoint &kpUn2 = pKF2->m_vKeysUn[i2];
			obs2 << kpUn2.pt.x, kpUn2.pt.y;

			g2o::EdgeInverseSim3ProjectXYZ* e21 = new g2o::EdgeInverseSim3ProjectXYZ();

			e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1)));
			e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
			e21->setMeasurement(obs2);
			float invSigmaSquare2 = pKF2->m_pPLevelInfo->m_vInvLevelSigma2[kpUn2.octave];
			e21->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare2);

			g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
			e21->setRobustKernel(rk2);
			rk2->setDelta(deltaHuber);
			optimizer.addEdge(e21);

			vpEdges12.push_back(e12);
			vpEdges21.push_back(e21);
			vnIndexEdge.push_back(i);
		}
		 
		optimizer.initializeOptimization();
		optimizer.optimize(5);
		 
		int nBad = 0;
		for (size_t i = 0; i < vpEdges12.size(); i++) {

			g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
			g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
			if (!e12 || !e21)
				continue;

			// outliers
			if (e12->chi2() > th2 || e21->chi2() > th2) {
				size_t idx = vnIndexEdge[i];
				vpMatches12[idx] = static_cast<MapPoint*>(NULL);
				optimizer.removeEdge(e12);
				optimizer.removeEdge(e21);
				vpEdges12[i] = static_cast<g2o::EdgeSim3ProjectXYZ*>(NULL);
				vpEdges21[i] = static_cast<g2o::EdgeInverseSim3ProjectXYZ*>(NULL);
				nBad++;
			}
		}
		 
		int nMoreIterations;
		if (nBad > 0)
			nMoreIterations = 10;
		else
			nMoreIterations = 5;
		 
		if (nCorrespondences - nBad < 10)
			return 0;
		 
		optimizer.initializeOptimization();
		optimizer.optimize(nMoreIterations);

		int nIn = 0;
		for (size_t i = 0; i < vpEdges12.size(); i++) {

			g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
			g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
			if (!e12 || !e21)
				continue;

			if (e12->chi2() > th2 || e21->chi2()>th2) {
				size_t idx = vnIndexEdge[i];
				vpMatches12[idx] = static_cast<MapPoint*>(NULL);
			}
			else
				nIn++;
		}
		 
		g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
		g2oS12 = vSim3_recov->estimate();
		 
		return nIn;
	}

	void Optimizer::OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF, const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
		const LoopClosing::KeyFrameAndPose &CorrectedSim3, const map<KeyFrame *, set<KeyFrame *> > &LoopConnections, CovisibilityGraph* pCoGraph
		, SpanningTree* pSpanTree) {
		 
		g2o::SparseOptimizer optimizer;
		optimizer.setVerbose(false);  
		 
		g2o::BlockSolver_7_3::LinearSolverType * linearSolver;
		linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();

		g2o::BlockSolver_7_3 * solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
		g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

		solver->setUserLambdaInit(1e-16);
		optimizer.setAlgorithm(solver);

		const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
		const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

		const unsigned int nMaxKFid = pMap->GetMaxKFid();
		 
		vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid + 1);   
		vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid + 1);  
		vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid + 1);

		const int minFeat = 100;
		 
		for (size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
			KeyFrame* pKF = vpKFs[i];
			if (pKF->isBad())
				continue;
			 
			g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

			const int nIDi = pKF->m_nKFId;

			LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);
			 
			if (it != CorrectedSim3.end()) {
				vScw[nIDi] = it->second;
				VSim3->setEstimate(it->second);  
			}
			else {
				Eigen::Matrix<double, 3, 3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
				Eigen::Matrix<double, 3, 1> tcw = Converter::toVector3d(pKF->GetTranslation());
				g2o::Sim3 Siw(Rcw, tcw, 1.0);
				vScw[nIDi] = Siw;
				VSim3->setEstimate(Siw);
			}
			 
			if (pKF == pLoopKF)
				VSim3->setFixed(true);
			 
			VSim3->setId(nIDi);
			VSim3->setMarginalized(false);  
			VSim3->_fix_scale = true;

			optimizer.addVertex(VSim3);

			vpVertices[nIDi] = VSim3;
		}


		set<pair<long unsigned int, long unsigned int> > sInsertedEdges;

		const Eigen::Matrix<double, 7, 7> matLambda = Eigen::Matrix<double, 7, 7>::Identity();
		 
		for (map<KeyFrame *, set<KeyFrame *> >::const_iterator mit = LoopConnections.begin(), mend = LoopConnections.end(); mit != mend; mit++) {

			KeyFrame* pKF = mit->first;
			const long unsigned int nIDi = pKF->m_nKFId;
			const set<KeyFrame*> &spConnections = mit->second;
			const g2o::Sim3 Siw = vScw[nIDi];
			const g2o::Sim3 Swi = Siw.inverse();

			for (set<KeyFrame*>::const_iterator sit = spConnections.begin(), send = spConnections.end(); sit != send; sit++) {

				const long unsigned int nIDj = (*sit)->m_nKFId;
				 
				if ((nIDi != pCurKF->m_nKFId || nIDj != pLoopKF->m_nKFId) && pCoGraph->GetWeight(pKF, *sit) < minFeat)
					continue;
				 
				const g2o::Sim3 Sjw = vScw[nIDj];
				const g2o::Sim3 Sji = Sjw * Swi;
				 
				g2o::EdgeSim3* e = new g2o::EdgeSim3();
				e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
				e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
				 
				e->setMeasurement(Sji);
				 
				e->information() = matLambda;

				optimizer.addEdge(e);

				sInsertedEdges.insert(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));
			}
		}
		 
		for (size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
			KeyFrame* pKF = vpKFs[i];

			const int nIDi = pKF->m_nKFId;
			 
			g2o::Sim3 Swi;
			LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);
			if (iti != NonCorrectedSim3.end())
				Swi = (iti->second).inverse();
			else
				Swi = vScw[nIDi].inverse();
			 
			KeyFrame* pParentKF = pSpanTree->GetParent(pKF);
			 
			if (pParentKF) {
				int nIDj = pParentKF->m_nKFId;

				g2o::Sim3 Sjw;

				LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

				if (itj != NonCorrectedSim3.end())
					Sjw = itj->second;
				else
					Sjw = vScw[nIDj];

				g2o::Sim3 Sji = Sjw * Swi;

				g2o::EdgeSim3* e = new g2o::EdgeSim3();
				e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
				e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
				e->setMeasurement(Sji);

				e->information() = matLambda;
				optimizer.addEdge(e);
			}
			 
			const set<KeyFrame*> sLoopEdges = pSpanTree->GetLoopEdges(pKF);
			for (set<KeyFrame*>::const_iterator sit = sLoopEdges.begin(), send = sLoopEdges.end(); sit != send; sit++) {
				KeyFrame* pLKF = *sit; 
				if (pLKF->m_nKFId < pKF->m_nKFId) {

					g2o::Sim3 Slw;

					LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);

					if (itl != NonCorrectedSim3.end())
						Slw = itl->second;
					else
						Slw = vScw[pLKF->m_nKFId];

					g2o::Sim3 Sli = Slw * Swi;
					g2o::EdgeSim3* el = new g2o::EdgeSim3();
					el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pLKF->m_nKFId)));
					el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
					el->setMeasurement(Sli);
					el->information() = matLambda;
					optimizer.addEdge(el);
				}
			}
			 
			const vector<KeyFrame*> vpConnectedKFs = pCoGraph->GetCovisiblesByWeight(pKF, minFeat);

			for (vector<KeyFrame*>::const_iterator vit = vpConnectedKFs.begin(); vit != vpConnectedKFs.end(); vit++) {

				KeyFrame* pKFn = *vit; 
				if (pKFn && pKFn != pParentKF && !pSpanTree->hasChild(pKFn, pKF) && !sLoopEdges.count(pKFn)) {
					 
					if (!pKFn->isBad() && pKFn->m_nKFId < pKF->m_nKFId) {
						 
						if (sInsertedEdges.count(make_pair(min(pKF->m_nKFId, pKFn->m_nKFId), max(pKF->m_nKFId, pKFn->m_nKFId))))
							continue;

						g2o::Sim3 Snw;

						LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

						if (itn != NonCorrectedSim3.end())
							Snw = itn->second;
						else
							Snw = vScw[pKFn->m_nKFId];

						g2o::Sim3 Sni = Snw * Swi;

						g2o::EdgeSim3* en = new g2o::EdgeSim3();
						en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFn->m_nKFId)));
						en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
						en->setMeasurement(Sni);
						en->information() = matLambda;
						optimizer.addEdge(en);
					}
				}
			}
		}


		// Optimize!
		optimizer.initializeOptimization();
		optimizer.optimize(20);

		unique_lock<mutex> lock(pMap->m_MutexMapUpdate);

		// SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1] 
		for (size_t i = 0; i < vpKFs.size(); i++) {
			KeyFrame* pKFi = vpKFs[i];

			const int nIDi = pKFi->m_nKFId;

			g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
			g2o::Sim3 CorrectedSiw = VSim3->estimate();
			vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
			Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
			Eigen::Vector3d eigt = CorrectedSiw.translation();
			double s = CorrectedSiw.scale();

			eigt *= (1. / s); //[R t/s;0 1]

			cv::Mat Tiw = Converter::toCvSE3(eigR, eigt);

			pKFi->SetPose(Tiw);

			pMap->addModifiedKeyFrame(pKFi);
		}
		 
		for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
			MapPoint* pMP = vpMPs[i];

			if (pMP->isBad())
				continue;

			int nIDr;
			if (pMP->m_nCorrectedByKF == pCurKF->m_nKFId) {
				nIDr = pMP->m_nCorrectedReference;
			}
			else {
				KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();
				nIDr = pRefKF->m_nKFId;
			}

			g2o::Sim3 Srw = vScw[nIDr];
			g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

			cv::Mat P3Dw = pMP->GetWorldPos();
			Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(P3Dw);
			Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

			cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
			pMP->SetWorldPos(cvCorrectedP3Dw);

			pMP->UpdateNormalAndDepth();
		}
	}

	void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust) {
		vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
		vector<MapPoint*> vpMP = pMap->GetAllMapPoints();

		BundleAdjustment(pMap, vpKFs, vpMP, nIterations, pbStopFlag, nLoopKF, bRobust);
	}

	void Optimizer::BundleAdjustment(Map* pMap, const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP,
		int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
	{ 
		vector<bool> vbNotIncludedMP;
		vbNotIncludedMP.resize(vpMP.size());
		 
		g2o::SparseOptimizer optimizer;
		g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

		linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

		g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

		g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
		optimizer.setAlgorithm(solver);

		if (pbStopFlag)
			optimizer.setForceStopFlag(pbStopFlag);

		long unsigned int maxKFid = 0;
		 
		for (size_t i = 0; i < vpKFs.size(); i++) {

			KeyFrame* pKF = vpKFs[i];
			if (pKF->isBad())
				continue;
			g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
			vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
			vSE3->setId(pKF->m_nKFId);
			vSE3->setFixed(pKF->m_nKFId == 0);
			optimizer.addVertex(vSE3);
			if (pKF->m_nKFId > maxKFid)
				maxKFid = pKF->m_nKFId;
		}

		const float thHuber2D = sqrt(5.99);
		const float thHuber3D = sqrt(7.815);

		// Set MapPoint vertices
		for (size_t i = 0; i<vpMP.size(); i++) {
			MapPoint* pMP = vpMP[i];
			if (pMP->isBad())
				continue;
			g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
			vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
			const int id = pMP->m_nMPId + maxKFid + 1;
			vPoint->setId(id);
			vPoint->setMarginalized(true); 
			//vPoint->setFixed(true);
			optimizer.addVertex(vPoint);

			const map<KeyFrame*, size_t> observations = pMP->GetObservations();

			int nEdges = 0; 
			for (map<KeyFrame*, size_t>::const_iterator mit = observations.begin(); mit != observations.end(); mit++) {

				KeyFrame* pKF = mit->first;
				if (pKF->isBad() || pKF->m_nKFId > maxKFid)
					continue;

				nEdges++;

				const cv::KeyPoint &kpUn = pKF->m_vKeysUn[mit->second];

				if (pKF->m_vuRight[mit->second] < 0) {
					Eigen::Matrix<double, 2, 1> obs;
					obs << kpUn.pt.x, kpUn.pt.y;

					g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

					e->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(id)));
					e->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->m_nKFId)));

					e->setMeasurement(obs);

					const float &invSigma2 = pKF->m_pPLevelInfo->m_vInvLevelSigma2[kpUn.octave];
					e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

					if (bRobust) {
						g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
						rk->setDelta(thHuber2D);
					}

					e->fx = pKF->m_pCameraInfo->m_fx;
					e->fy = pKF->m_pCameraInfo->m_fy;
					e->cx = pKF->m_pCameraInfo->m_cx;
					e->cy = pKF->m_pCameraInfo->m_cy;

					optimizer.addEdge(e);
				}
				else {
					Eigen::Matrix<double, 3, 1> obs;
					const float kp_ur = pKF->m_vuRight[mit->second];
					obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

					g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

					e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
					e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->m_nKFId)));
					e->setMeasurement(obs);
					const float &invSigma2 = pKF->m_pPLevelInfo->m_vInvLevelSigma2[kpUn.octave];
					Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
					e->setInformation(Info);

					if (bRobust) {
						g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
						rk->setDelta(thHuber3D);
					}

					e->fx = pKF->m_pCameraInfo->m_fx;
					e->fy = pKF->m_pCameraInfo->m_fy;
					e->cx = pKF->m_pCameraInfo->m_cx;
					e->cy = pKF->m_pCameraInfo->m_cy;
					e->bf = pKF->m_pCameraInfo->m_bf;

					optimizer.addEdge(e);
				}

			}
			 
			if (nEdges == 0) {
				optimizer.removeVertex(vPoint);
				vbNotIncludedMP[i] = true;
			}
			else {
				vbNotIncludedMP[i] = false;
			}
		}
		 
		optimizer.initializeOptimization();
		optimizer.optimize(nIterations);
		  
		for (size_t i = 0; i < vpKFs.size(); i++) {
			KeyFrame* pKF = vpKFs[i];
			if (pKF->isBad())
				continue;
			g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->m_nKFId));
			g2o::SE3Quat SE3quat = vSE3->estimate();
			if (nLoopKF == 0) {
				pKF->SetPose(Converter::toCvMat(SE3quat));

				pMap->addModifiedKeyFrame(pKF);
			}
			else {
				pKF->m_TcwGBA.create(4, 4, CV_32F);
				Converter::toCvMat(SE3quat).copyTo(pKF->m_TcwGBA);
				pKF->m_nBAGlobalForKF = nLoopKF;
			}
		}
		 
		for (size_t i = 0; i < vpMP.size(); i++) {
			if (vbNotIncludedMP[i])
				continue;

			MapPoint* pMP = vpMP[i];

			if (pMP->isBad())
				continue;
			g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->m_nMPId + maxKFid + 1));

			if (nLoopKF == 0) {
				pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
				pMP->UpdateNormalAndDepth();
			}
			else{
				pMP->m_PosGBA.create(3, 1, CV_32F);
				Converter::toCvMat(vPoint->estimate()).copyTo(pMP->m_PosGBA);
				pMP->m_nBAGlobalForKF = nLoopKF;
			}
		}

	}
} // namespace SLAMRecon
