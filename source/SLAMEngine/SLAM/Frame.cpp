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

#include "Frame.h"
#include "MapPoint.h"
#include "Converter.h"
#include "KeyFrame.h"

using namespace std;

namespace SLAMRecon {

	long unsigned int Frame::m_nFNextId = 0;
	bool Frame::m_bInitialComputations = true;
	PyramidLevelInfo*  Frame::m_pPLevelInfo;
	CameraInfo*  Frame::m_pCameraInfo;
	GridInfo* Frame::m_pGridInfo;

	Frame::Frame() {}

	Frame::Frame(const Frame &frame) 
		: m_nFId(frame.m_nFId), m_rgbImg(frame.m_rgbImg.clone()), m_pReferenceKF(frame.m_pReferenceKF), m_pORBvocabulary(frame.m_pORBvocabulary), m_ORBextractor(frame.m_ORBextractor),
		m_nKeys(frame.m_nKeys), m_vKeys(frame.m_vKeys), m_vKeysUn(frame.m_vKeysUn), m_vpMapPoints(frame.m_vpMapPoints), m_vbOutlier(frame.m_vbOutlier),
		m_Descriptors(frame.m_Descriptors.clone()), m_BowVec(frame.m_BowVec), m_FeatVec(frame.m_FeatVec), m_vfDepth(frame.m_vfDepth), m_vuRight(frame.m_vuRight), m_fThDepth(frame.m_fThDepth)
	{
		for (int i = 0; i < FRAME_GRID_COLS; i++)
			for (int j = 0; j < FRAME_GRID_ROWS; j++)
				m_Grid[i][j] = frame.m_Grid[i][j];

		if (!frame.m_Transformation.empty())
			SetPose(frame.m_Transformation);
	}

	Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, ORBextractor* extractor, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, float m_bf, float m_fThDepth)
		:m_ORBextractor(extractor), m_pORBvocabulary(voc), m_fThDepth(m_fThDepth)
	{
		// Frame ID
		m_nFId = m_nFNextId++;

		// This is done only for the first Frame(or after a change in the calibration) 
		if (m_bInitialComputations) {
			 
			m_pPLevelInfo = new PyramidLevelInfo;
			m_pPLevelInfo->m_nScaleLevels = extractor->GetLevels();
			m_pPLevelInfo->m_fScaleFactor = extractor->GetScaleFactor();
			m_pPLevelInfo->m_fLogScaleFactor = log(m_pPLevelInfo->m_fScaleFactor);
			m_pPLevelInfo->m_vScaleFactors = extractor->GetScaleFactors();
			m_pPLevelInfo->m_vInvScaleFactors = extractor->GetInverseScaleFactors();
			m_pPLevelInfo->m_vLevelSigma2 = extractor->GetScaleSigmaSquares();
			m_pPLevelInfo->m_vInvLevelSigma2 = extractor->GetInverseScaleSigmaSquares();
			 
			m_pCameraInfo = new CameraInfo;
			m_pCameraInfo->m_K = K.clone();
			m_pCameraInfo->m_DistCoef = distCoef.clone();
			m_pCameraInfo->m_bf = m_bf;
			 
			ComputeImageBounds(imGray); 

			m_pCameraInfo->m_fx = K.at<float>(0, 0);
			m_pCameraInfo->m_fy = K.at<float>(1, 1);
			m_pCameraInfo->m_cx = K.at<float>(0, 2);
			m_pCameraInfo->m_cy = K.at<float>(1, 2);
			m_pCameraInfo->m_invfx = 1.0f / m_pCameraInfo->m_fx;
			m_pCameraInfo->m_invfy = 1.0f / m_pCameraInfo->m_fy;
			 
			m_pGridInfo = new GridInfo;
			m_pGridInfo->m_fGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / (m_pCameraInfo->m_nMaxX - m_pCameraInfo->m_nMinX);
			m_pGridInfo->m_fGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / (m_pCameraInfo->m_nMaxY - m_pCameraInfo->m_nMinY);

			m_bInitialComputations = false;  
		}
		 
		ExtractORB(imGray); 

		if (m_vKeys.empty())
			return;
		m_nKeys = m_vKeys.size();
		 
		UndistortKeyPoints();
		 
		ComputeDepthFromRGBD(imDepth);
		 
		m_vpMapPoints = vector<MapPoint*>(m_nKeys, static_cast<MapPoint*>(NULL)); 
		m_vbOutlier = vector<bool>(m_nKeys, false);
		 
		AssignFeaturesToGrid();
	}
	
	Frame::~Frame() {

	}

	void Frame::setRGBImg(cv::Mat rgbImg) {
		m_rgbImg = rgbImg.clone();
	}

	void Frame::ExtractORB(const cv::Mat &im) {
		m_ORBextractor->detect(im, cv::Mat(), m_vKeys, m_Descriptors);
	}

	void Frame::ComputeDepthFromRGBD(const cv::Mat &imDepth) {
		m_vfDepth = vector<float>(m_nKeys, -1);  
		m_vuRight = vector<float>(m_nKeys, -1);

		for (int i = 0; i < m_nKeys; i++) { 
			const cv::KeyPoint &kp = m_vKeys[i]; 
			const cv::KeyPoint &kpU = m_vKeysUn[i];

			const float &v = kp.pt.y;
			const float &u = kp.pt.x;

			const float d = imDepth.at<float>(v, u);

			if (d > 0) {
				m_vfDepth[i] = d;
				m_vuRight[i] = kpU.pt.x - m_pCameraInfo->m_bf / d;
			}
		}
	}

	void Frame::SetPose(cv::Mat Transformation) {
		m_Transformation = Transformation.clone();
		UpdatePoseMatrices();
	}

	void Frame::UpdatePoseMatrices() {
		m_R = m_Transformation.rowRange(0, 3).colRange(0, 3);
		m_t = m_Transformation.rowRange(0, 3).col(3); 
		m_C = -m_R.t()*m_t;
	}

	cv::Mat Frame::GetCameraCenter() {
		return m_C.clone();
	}

	cv::Mat Frame::GetRotation() {
		return m_R.clone();
	}

	cv::Mat Frame::GetTranslation() {
		return m_t.clone();
	}

	void Frame::ComputeBoW() {
		if (m_BowVec.empty()) { 
			vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(m_Descriptors);
			m_pORBvocabulary->transform(vCurrentDesc, m_BowVec, m_FeatVec, 4);
		}
	}

	cv::Mat Frame::ComputeWorldPos(const int &i)
	{
		const float z = m_vfDepth[i];
		if (z > 0) { 
			const float u = m_vKeysUn[i].pt.x;
			const float v = m_vKeysUn[i].pt.y;
			 
			const float x = (u - m_pCameraInfo->m_cx)*z*m_pCameraInfo->m_invfx;
			const float y = (v - m_pCameraInfo->m_cy)*z*m_pCameraInfo->m_invfy;
			cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);

			// m_R * x3Dw + m_t = x3Dc -> x3Dw = m_R.t() * x3Dc - m_R.t() * m_t -> x3Dw = m_R.t() * x3Dc - m_R.t() * m_t
			return m_R.t()*x3Dc + m_C;
		}
		else
			return cv::Mat();
	}

	bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit) {
		pMP->m_bTrackInView = false;

		 
		cv::Mat P = pMP->GetWorldPos();  
		const cv::Mat Pc = m_R*P + m_t;  

		
		const float &PcX = Pc.at<float>(0);
		const float &PcY = Pc.at<float>(1);
		const float &PcZ = Pc.at<float>(2);
		 
		if (PcZ < 0.0f)
			return false;
		 
		const float invz = 1.0f / PcZ;
		const float u = m_pCameraInfo->m_fx*PcX*invz + m_pCameraInfo->m_cx;
		const float v = m_pCameraInfo->m_fy*PcY*invz + m_pCameraInfo->m_cy;
		 
		if (u < m_pCameraInfo->m_nMinX || u > m_pCameraInfo->m_nMaxX)
			return false;
		if (v < m_pCameraInfo->m_nMinY || v > m_pCameraInfo->m_nMaxY)
			return false;

		 
		const float maxDistance = pMP->GetMaxDistanceInvariance();
		const float minDistance = pMP->GetMinDistanceInvariance();
		 
		const cv::Mat PO = P - m_C;  
		const float dist = cv::norm(PO);
		 
		if (dist<minDistance || dist>maxDistance)
			return false;
		 
		cv::Mat Pn = pMP->GetNormal();
		 
		const float viewCos = PO.dot(Pn) / dist;
		 
		if (viewCos < viewingCosLimit)
			return false;
		 
		const int nPredictedLevel = pMP->PredictScale(dist, m_pPLevelInfo->m_fLogScaleFactor); 
		pMP->m_bTrackInView = true;  
		pMP->m_TrackProjX = u;  
		pMP->m_TrackProjY = v;
		pMP->m_nTrackScaleLevel = nPredictedLevel;
		pMP->m_TrackViewCos = viewCos; 

		return true;
	}

	vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const {
		 
		vector<size_t> vIndices; 
		vIndices.reserve(m_nKeys);
		 
		const int nMinCellX = max(0, (int)floor((x - m_pCameraInfo->m_nMinX - r)*m_pGridInfo->m_fGridElementWidthInv)); 
		if (nMinCellX >= FRAME_GRID_COLS)
			return vIndices;

		const int nMaxCellX = min((int)FRAME_GRID_COLS - 1, (int)ceil((x - m_pCameraInfo->m_nMinX + r)*m_pGridInfo->m_fGridElementWidthInv));
		if (nMaxCellX < 0)
			return vIndices;

		const int nMinCellY = max(0, (int)floor((y - m_pCameraInfo->m_nMinY - r)*m_pGridInfo->m_fGridElementHeightInv));
		if (nMinCellY >= FRAME_GRID_ROWS)
			return vIndices;

		const int nMaxCellY = min((int)FRAME_GRID_ROWS - 1, (int)ceil((y - m_pCameraInfo->m_nMinY + r)*m_pGridInfo->m_fGridElementHeightInv));
		if (nMaxCellY < 0)
			return vIndices;
		 
		const bool bCheckLevels = (minLevel>0) || (maxLevel >= 0);
		 
		for (int ix = nMinCellX; ix <= nMaxCellX; ix++) { 
			for (int iy = nMinCellY; iy <= nMaxCellY; iy++) { 
				 
				const vector<size_t> vCell = m_Grid[ix][iy];
				if (vCell.empty())
					continue;
				 
				for (size_t j = 0, jend = vCell.size(); j < jend; j++) {

					const cv::KeyPoint &kpUn = m_vKeysUn[vCell[j]];
					 
					if (bCheckLevels) { 
						if (kpUn.octave < minLevel)
							continue; 
						if (maxLevel >= 0)
							if (kpUn.octave > maxLevel)
								continue;
					}
					 
					const float distx = kpUn.pt.x - x;
					const float disty = kpUn.pt.y - y;
					if (fabs(distx) < r && fabs(disty) < r)
						vIndices.push_back(vCell[j]);
				}
			}
		}
		return vIndices;
	}

	bool Frame::IsInImage(const float &x, const float &y) const {
		return (x >= m_pCameraInfo->m_nMinX && x < m_pCameraInfo->m_nMaxX && y >= m_pCameraInfo->m_nMinY && y < m_pCameraInfo->m_nMaxY);
	}
	 
	void Frame::UndistortKeyPoints() { 
		if (m_pCameraInfo->m_DistCoef.at<float>(0) == 0.0) {
			m_vKeysUn = m_vKeys;
			return;
		}

		// Fill matrix with points 
		cv::Mat mat(m_nKeys, 2, CV_32F);
		for (int i = 0; i < m_nKeys; i++) {
			mat.at<float>(i, 0) = m_vKeys[i].pt.x;
			mat.at<float>(i, 1) = m_vKeys[i].pt.y;
		}

		// Undistort points 
		mat = mat.reshape(2);
		cv::undistortPoints(mat, mat, m_pCameraInfo->m_K, m_pCameraInfo->m_DistCoef, cv::Mat(), m_pCameraInfo->m_K);
		mat = mat.reshape(1);
		 
		m_vKeysUn.resize(m_nKeys);
		for (int i = 0; i < m_nKeys; i++) {
			cv::KeyPoint kp = m_vKeys[i];
			kp.pt.x = mat.at<float>(i, 0);
			kp.pt.y = mat.at<float>(i, 1);
			m_vKeysUn[i] = kp;
		}
	}

	void Frame::ComputeImageBounds(const cv::Mat &im) { 
		if (m_pCameraInfo->m_DistCoef.at<float>(0) != 0.0) {
			 
			cv::Mat mat(4, 2, CV_32F);
			mat.at<float>(0, 0) = 0.0;
			mat.at<float>(0, 1) = 0.0;

			mat.at<float>(1, 0) = im.cols;
			mat.at<float>(1, 1) = 0.0;

			mat.at<float>(2, 0) = 0.0;
			mat.at<float>(2, 1) = im.rows;

			mat.at<float>(3, 0) = im.cols;
			mat.at<float>(3, 1) = im.rows;
			 
			mat = mat.reshape(2);
			cv::undistortPoints(mat, mat, m_pCameraInfo->m_K, m_pCameraInfo->m_DistCoef, cv::Mat(), m_pCameraInfo->m_K);
			mat = mat.reshape(1);

			m_pCameraInfo->m_nMinX = min(mat.at<float>(0, 0), mat.at<float>(2, 0));
			m_pCameraInfo->m_nMaxX = max(mat.at<float>(1, 0), mat.at<float>(3, 0));
			m_pCameraInfo->m_nMinY = min(mat.at<float>(0, 1), mat.at<float>(1, 1));
			m_pCameraInfo->m_nMaxY = max(mat.at<float>(2, 1), mat.at<float>(3, 1));
		}
		else {
			m_pCameraInfo->m_nMinX = 0.0f;
			m_pCameraInfo->m_nMaxX = im.cols;
			m_pCameraInfo->m_nMinY = 0.0f;
			m_pCameraInfo->m_nMaxY = im.rows;
		}
	}

	void Frame::AssignFeaturesToGrid() { 
		int nReserve = 0.5f*m_nKeys / (FRAME_GRID_COLS*FRAME_GRID_ROWS); 

		for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
			for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++)
				m_Grid[i][j].reserve(nReserve);
		 
		for (int i = 0; i < m_nKeys; i++) {
			const cv::KeyPoint &kp = m_vKeysUn[i];
			int nGridPosX, nGridPosY;
			if (PosInGrid(kp, nGridPosX, nGridPosY))
				m_Grid[nGridPosX][nGridPosY].push_back(i);
		}
	}

	bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY) { 
		posX = round((kp.pt.x - m_pCameraInfo->m_nMinX)*m_pGridInfo->m_fGridElementWidthInv);
		posY = round((kp.pt.y - m_pCameraInfo->m_nMinY)*m_pGridInfo->m_fGridElementHeightInv);
		 
		if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 || posY >= FRAME_GRID_ROWS)
			return false;

		return true;
	}

} // namespace SLAMRecon