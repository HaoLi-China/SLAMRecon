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

#ifndef SIM3SOLVER_H
#define SIM3SOLVER_H

#include "KeyFrame.h"
#include "MapPoint.h"

#include <vector>

namespace SLAMRecon {

	class Sim3Solver {

	public:
		Sim3Solver(KeyFrame* pKF1, KeyFrame* pKF2, const std::vector<MapPoint*> &vpMatched12);

		void SetRansacParameters(double probability = 0.99, int minInliers = 6, int maxIterations = 300);

		cv::Mat iterate(int nIterations, bool &bNoMore, std::vector<bool> &vbInliers, int &nInliers);

		cv::Mat GetEstimatedRotation();
		cv::Mat GetEstimatedTranslation();

	protected:
		void FromCameraToImage(const std::vector<cv::Mat> &vP3Dc, std::vector<cv::Mat> &vP2D, cv::Mat K);

		void ComputeSim3(cv::Mat &P1, cv::Mat &P2);

		void ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C);

		void CheckInliers();

		void Project(const std::vector<cv::Mat> &vP3Dw, std::vector<cv::Mat> &vP2D, cv::Mat Tcw, cv::Mat K);

	protected:
		
		// KeyFrames and matches
		KeyFrame* m_pKF1;
		KeyFrame* m_pKF2;

		std::vector<MapPoint*> m_vpMatches12;

		std::vector<MapPoint*> m_vpMapPoints1;
		std::vector<MapPoint*> m_vpMapPoints2;

		std::vector<size_t> m_vnIndices1;

		std::vector<cv::Mat> m_vX3Dc1;
		std::vector<cv::Mat> m_vX3Dc2;

		std::vector<cv::Mat> m_vP1im1;
		std::vector<cv::Mat> m_vP2im2;

		std::vector<size_t> m_vnMaxError1;
		std::vector<size_t> m_vnMaxError2;
		
		std::vector<size_t> m_vAllIndices;
		 
		int m_N1;

		// Calibration
		cv::Mat m_K1;
		cv::Mat m_K2;
		 
		int m_N;

		// Ransac parameters
		double m_RansacProb;
		int m_RansacMinInliers;
		int m_RansacMaxIts;
		 
		// Current Ransac State
		int m_nIterations;  
		cv::Mat m_BestT12; 
		cv::Mat m_BestR12;  
		cv::Mat m_Bestt12;  
		std::vector<bool> m_vbBestInliers;  
		int m_nBestInliers;   

		// Current Estimation
		cv::Mat m_R12i;  
		cv::Mat m_t12i;  
		cv::Mat m_T12i;
		cv::Mat m_T21i;  
		std::vector<bool> m_vbInliersi;  
		int m_nInliersi;  
	};

} // namespace SLAMRecon

#endif // SIM3SOLVER_H