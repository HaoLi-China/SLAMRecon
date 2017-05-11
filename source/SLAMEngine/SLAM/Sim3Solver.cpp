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

#include "Sim3Solver.h"

using namespace std;

namespace SLAMRecon {

	Sim3Solver::Sim3Solver(KeyFrame* pKF1, KeyFrame* pKF2, const std::vector<MapPoint*> &vpMatched12) {

		m_pKF1 = pKF1;
		m_pKF2 = pKF2;

		m_N1 = vpMatched12.size();

		vector<MapPoint*> vpKeyFrameMP1 = pKF1->GetMapPointMatches();

		
		m_vpMapPoints1.reserve(m_N1);
		m_vpMapPoints2.reserve(m_N1);

		m_vpMatches12 = vpMatched12;


		m_vnIndices1.reserve(m_N1);

		m_vX3Dc1.reserve(m_N1);
		m_vX3Dc2.reserve(m_N1);

		cv::Mat Rcw1 = pKF1->GetRotation();
		cv::Mat tcw1 = pKF1->GetTranslation();
		cv::Mat Rcw2 = pKF2->GetRotation();
		cv::Mat tcw2 = pKF2->GetTranslation();

		m_vAllIndices.reserve(m_N1);


		size_t idx = 0;
		for (int i1 = 0; i1 < m_N1; i1++) {

			if (vpMatched12[i1]) {

				MapPoint* pMP1 = vpKeyFrameMP1[i1];
				MapPoint* pMP2 = vpMatched12[i1];

				if (!pMP1)
					continue;

				if (pMP1->isBad() || pMP2->isBad())
					continue;

				int indexKF1 = pMP1->GetIndexInKeyFrame(pKF1);
				int indexKF2 = pMP2->GetIndexInKeyFrame(pKF2);

				if (indexKF1 < 0 || indexKF2 < 0)
					continue;

				const cv::KeyPoint &kp1 = pKF1->m_vKeysUn[indexKF1];
				const cv::KeyPoint &kp2 = pKF2->m_vKeysUn[indexKF2];


				const float sigmaSquare1 = pKF1->m_pPLevelInfo->m_vLevelSigma2[kp1.octave];
				const float sigmaSquare2 = pKF2->m_pPLevelInfo->m_vLevelSigma2[kp2.octave];
				m_vnMaxError1.push_back(9.210*sigmaSquare1);
				m_vnMaxError2.push_back(9.210*sigmaSquare2);
				
				m_vpMapPoints1.push_back(pMP1);
				m_vpMapPoints2.push_back(pMP2);

				m_vnIndices1.push_back(i1);

				cv::Mat X3D1w = pMP1->GetWorldPos();
				m_vX3Dc1.push_back(Rcw1*X3D1w + tcw1);

				cv::Mat X3D2w = pMP2->GetWorldPos();
				m_vX3Dc2.push_back(Rcw2*X3D2w + tcw2);

				m_vAllIndices.push_back(idx);
				idx++;
			}
		}

		m_K1 = pKF1->m_pCameraInfo->m_K;
		m_K2 = pKF2->m_pCameraInfo->m_K;

		FromCameraToImage(m_vX3Dc1, m_vP1im1, m_K1);
		FromCameraToImage(m_vX3Dc2, m_vP2im2, m_K2);

		SetRansacParameters();

	}

	void Sim3Solver::SetRansacParameters(double probability, int minInliers, int maxIterations) {
		m_RansacProb = probability;
		m_RansacMinInliers = minInliers;
		m_RansacMaxIts = maxIterations;

		m_N = m_vpMapPoints1.size(); 

		m_vbInliersi.resize(m_N);

		float epsilon = (float)m_RansacMinInliers / m_N;

		int nIterations;
		if (m_RansacMinInliers == m_N)
			nIterations = 1;
		else
			nIterations = ceil(log(1 - m_RansacProb) / log(1 - pow(epsilon, 3)));
		m_RansacMaxIts = max(1, min(nIterations, m_RansacMaxIts));

		m_nIterations = 0;
	}

	cv::Mat Sim3Solver::iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers) {

		bNoMore = false;

		vbInliers = vector<bool>(m_N1, false);
		nInliers = 0;

		if (m_N < m_RansacMinInliers) {
			bNoMore = true;
			return cv::Mat();
		}

		vector<size_t> vAvailableIndices;

		cv::Mat P3Dc1i(3, 3, CV_32F);
		cv::Mat P3Dc2i(3, 3, CV_32F);

		int nCurrentIterations = 0;

		while (m_nIterations < m_RansacMaxIts && nCurrentIterations < nIterations) {

			nCurrentIterations++;
			m_nIterations++;

			vAvailableIndices = m_vAllIndices;

			for (short i = 0; i < 3; ++i) {

				int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size() - 1);

				int idx = vAvailableIndices[randi];

				m_vX3Dc1[idx].copyTo(P3Dc1i.col(i));
				m_vX3Dc2[idx].copyTo(P3Dc2i.col(i));

				vAvailableIndices[idx] = vAvailableIndices.back();
				vAvailableIndices.pop_back();
			}
			
			ComputeSim3(P3Dc1i, P3Dc2i);

			CheckInliers();

			if (m_nInliersi >= m_nBestInliers) {

				m_vbBestInliers = m_vbInliersi;
				m_nBestInliers = m_nInliersi;
				m_BestT12 = m_T12i.clone();
				m_BestR12 = m_R12i.clone();
				m_Bestt12 = m_t12i.clone();

				if (m_nInliersi > m_RansacMinInliers) {
					nInliers = m_nInliersi;
					for (int i = 0; i < m_N; i++)
						if (m_vbInliersi[i])
							vbInliers[m_vnIndices1[i]] = true;
					return m_BestT12;
				}
			}
		}

		if (m_nIterations >= m_RansacMaxIts)
			bNoMore = true;
		
		return cv::Mat();
	}

	cv::Mat Sim3Solver::GetEstimatedRotation() {
		return m_BestR12.clone();
	}

	cv::Mat Sim3Solver::GetEstimatedTranslation() {
		return m_Bestt12.clone();
	}

	void Sim3Solver::FromCameraToImage(const std::vector<cv::Mat> &vP3Dc, std::vector<cv::Mat> &vP2D, cv::Mat K) {

		const float &fx = K.at<float>(0, 0);
		const float &fy = K.at<float>(1, 1);
		const float &cx = K.at<float>(0, 2);
		const float &cy = K.at<float>(1, 2);

		vP2D.clear();
		vP2D.reserve(vP3Dc.size());

		for (size_t i = 0, iend = vP3Dc.size(); i < iend; i++) {
			const float invz = 1 / (vP3Dc[i].at<float>(2));
			const float x = vP3Dc[i].at<float>(0)*invz;
			const float y = vP3Dc[i].at<float>(1)*invz;

			vP2D.push_back((cv::Mat_<float>(2, 1) << fx*x + cx, fy*y + cy));
		}
	}

	void Sim3Solver::ComputeSim3(cv::Mat &P1, cv::Mat &P2) {

		// Custom implementation of:
		// Horn 1987, Closed-form solution of absolute orientataion using unit quaternions

		cv::Mat Pr1(P1.size(), P1.type()); 
		cv::Mat Pr2(P2.size(), P2.type()); 

		cv::Mat O1(3, 1, Pr1.type()); 
		cv::Mat O2(3, 1, Pr2.type()); 

		ComputeCentroid(P1, Pr1, O1);
		ComputeCentroid(P2, Pr2, O2);

		cv::Mat M = Pr2*Pr1.t();


		double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

		cv::Mat N(4, 4, P1.type());

		N11 = M.at<float>(0, 0) + M.at<float>(1, 1) + M.at<float>(2, 2);
		N12 = M.at<float>(1, 2) - M.at<float>(2, 1);
		N13 = M.at<float>(2, 0) - M.at<float>(0, 2);
		N14 = M.at<float>(0, 1) - M.at<float>(1, 0);
		N22 = M.at<float>(0, 0) - M.at<float>(1, 1) - M.at<float>(2, 2);
		N23 = M.at<float>(0, 1) + M.at<float>(1, 0);
		N24 = M.at<float>(2, 0) + M.at<float>(0, 2);
		N33 = -M.at<float>(0, 0) + M.at<float>(1, 1) - M.at<float>(2, 2);
		N34 = M.at<float>(1, 2) + M.at<float>(2, 1);
		N44 = -M.at<float>(0, 0) - M.at<float>(1, 1) + M.at<float>(2, 2);

		N = (cv::Mat_<float>(4, 4) << N11, N12, N13, N14,
			N12, N22, N23, N24,
			N13, N23, N33, N34,
			N14, N24, N34, N44);


		// Step 4: Eigenvector of the highest eigenvalue

		cv::Mat eval, evec;

		cv::eigen(N, eval, evec); //evec[0] is the quaternion of the desired rotation

		cv::Mat vec(1, 3, evec.type());
		(evec.row(0).colRange(1, 4)).copyTo(vec); //extract imaginary part of the quaternion (sin*axis)

		// Rotation angle. sin is the norm of the imaginary part, cos is the real part
		double ang = atan2(norm(vec), evec.at<float>(0, 0));

		vec = 2 * ang*vec / norm(vec); //Angle-axis representation. quaternion angle is the half

		m_R12i.create(3, 3, P1.type());

		cv::Rodrigues(vec, m_R12i); // computes the rotation matrix from angle-axis

		// Step 5: Rotate set 2

		cv::Mat P3 = m_R12i*Pr2;

		// Step 6: Scale

		float m_s12i = 1.0f;

		// Step 7: Translation

		m_t12i.create(1, 3, P1.type());
		m_t12i = O1 - m_s12i*m_R12i*O2;

		// Step 8: Transformation

		// Step 8.1 T12
		m_T12i = cv::Mat::eye(4, 4, P1.type());

		cv::Mat sR = m_s12i*m_R12i;

		sR.copyTo(m_T12i.rowRange(0, 3).colRange(0, 3));
		m_t12i.copyTo(m_T12i.rowRange(0, 3).col(3));

		// Step 8.2 T21

		m_T21i = cv::Mat::eye(4, 4, P1.type());

		cv::Mat sRinv = (1.0 / m_s12i)*m_R12i.t();

		sRinv.copyTo(m_T21i.rowRange(0, 3).colRange(0, 3));
		cv::Mat tinv = -sRinv*m_t12i;
		tinv.copyTo(m_T21i.rowRange(0, 3).col(3));
	}

	void Sim3Solver::ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C) {

		cv::reduce(P, C, 1, CV_REDUCE_SUM);
		C = C / P.cols;

		for (int i = 0; i < P.cols; i++) {
			Pr.col(i) = P.col(i) - C;
		}
	}

	void Sim3Solver::CheckInliers() {

		vector<cv::Mat> vP1im2, vP2im1;

		Project(m_vX3Dc2, vP2im1, m_T12i, m_K1);

		Project(m_vX3Dc1, vP1im2, m_T21i, m_K2);

		m_nInliersi = 0;

		for (size_t i = 0; i < m_vP1im1.size(); i++) {

			cv::Mat dist1 = m_vP1im1[i] - vP2im1[i];
			cv::Mat dist2 = vP1im2[i] - m_vP2im2[i];

			const float err1 = dist1.dot(dist1);
			const float err2 = dist2.dot(dist2);

			if (err1 < m_vnMaxError1[i] && err2 < m_vnMaxError2[i]) {
				m_vbInliersi[i] = true;
				m_nInliersi++;
			}
			else
				m_vbInliersi[i] = false;
		}
	}

	void Sim3Solver::Project(const vector<cv::Mat> &vP3Dw, vector<cv::Mat> &vP2D, cv::Mat Tcw, cv::Mat K) {

		cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
		cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
		const float &fx = K.at<float>(0, 0);
		const float &fy = K.at<float>(1, 1);
		const float &cx = K.at<float>(0, 2);
		const float &cy = K.at<float>(1, 2);

		vP2D.clear();
		vP2D.reserve(vP3Dw.size());

		for (size_t i = 0, iend = vP3Dw.size(); i < iend; i++) {
			cv::Mat P3Dc = Rcw*vP3Dw[i] + tcw;
			const float invz = 1 / (P3Dc.at<float>(2));
			const float x = P3Dc.at<float>(0)*invz;
			const float y = P3Dc.at<float>(1)*invz;

			vP2D.push_back((cv::Mat_<float>(2, 1) << fx*x + cx, fy*y + cy));
		}
	}

} // namespace SLAMRecon