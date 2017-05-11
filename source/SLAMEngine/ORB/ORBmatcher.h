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

#ifndef _ORB_MATCHER_H
#define _ORB_MATCHER_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <vector>

#include "../SLAM/Frame.h"

#ifdef _WIN32
#include <stdint.h>
#else
#include <stdint-gcc.h>
#endif

namespace SLAMRecon {
	class Frame;

	class ORBmatcher
	{
		
	public:
		//ORBmatcher();
		ORBmatcher(float nnratio = 0.6, bool checkOri = true);
		~ORBmatcher();
		
		// TrackWithMotionModel，用于从Last Frame中进行track
		// 求LastFrame的MapPoint可以匹配到CurrentFrame上的点
		// th表示查找半径系数
		int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th);

		// 主要用于TrackLocalMap中，求vpMapPoints上可以匹配到Frame上的点
		// th表示查找半径系数
		int SearchByProjection(Frame &F, const vector<MapPoint*> &vpMapPoints, const float th = 3);

		// KeyFrame中的MapPoints是固定的，那么现在要求的是Frame相对KeyFrame的match的点的个数
		// 利用FeatureVector加速，对Frame和KeyFrame属于同一个NodeId的KeyPoint进行暴力查询对比，很大程度上减少查找的次数
		// 求Frame中在KeyFrame上匹配上的MapPoint，存储在vpMapPointMatches（顺序对应Frame中的KeyPoint顺序）变量中
		// 返回匹配的点的个数
		int SearchByBoW(KeyFrame *pKF, Frame &F, vector<MapPoint*> &vpMapPointMatches);

		// 求两个KeyFrame之间的匹配的点，返回值为vpMatches12匹配值以及return匹配的个数
		// 每个index上的MapPoint表示在KeyFrame2中与KeyFrame1在该index上匹配的MapPoint，需要满足一定的阈值
		// 主要用于Loop Closing的ComputeSim3方法
		// 利用FeatureVector加速，对两个KeyFrame属于同一个NodeId的KeyPoint进行暴力查询对比，很大程度上减少查找的次数
		int SearchByBoW(KeyFrame *pKF1, KeyFrame* pKF2, vector<MapPoint*> &vpMatches12);

		// 应用在relocalisation (Tracking)中
		// 为候选的KeyFrame查找更多的匹配的MapPoint
		// 在 Relocalization 中使用优化求解找到的inliers还太少，那就使用下面的方法在一个大点的window size找到更多的inliers
		// sAlreadyFound pnp相对该KeyFrame已经找到的
		int SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, const set<MapPoint*> &sAlreadyFound, const float th, const int ORBdist);

		

		

		

		// Project MapPoints seen in KeyFrame into the Frame and search matches.
		
		// Matching to triangulate new MapPoints. Check Epipolar Constraint.
		int SearchForTriangulation(KeyFrame *pKF1, KeyFrame* pKF2, cv::Mat F12, vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo);
		
		// Project MapPoints into KeyFrame and search for duplicated MapPoints.
		int Fuse(KeyFrame* pKF, const vector<MapPoint*> &vpMapPoints, const float th=3.0);


		
		// 计算两个ORB descriptor（256位）之间的汉明距离 
		static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

		// 主要用在Loop Closing的ComputeSim3
		// pKF1和pKF2是两个KeyFrame，vpMatches12是上面的SearchByBoW求出来的两个KeyFrame之间的MapPoint的关系，但是以及进行过滤过了，只保存了里面的正常值，去除了一些匹配的异常值
		// R12是KeyFrame1相对KeyFrame2的旋转矩阵，同理t12是平移向量，th则是阈值，关于查找半径
		// 
		// 该方法主要是通过求MapPoint在另一个KeyFrame上对应的KeyPoint，如果正好双向关联，
		// 即KeyFrame1的某个MapPoint在KeyFrame2上可能找到一个匹配的KeyPoint，这个KeyPoint肯定也对应一个MapPoint，这个MapPoint去KeyFrame1也能找到一个匹配的KeyPoint
		// 这个KeyPoint正好和最开始KeyFrame1上的那个MapPoint对应，就认为这一对MapPoint是匹配上的
		// 当然这里主要计算新的MapPoint匹配，所以传入的匹配对不考虑
		// 返回的是新加入的匹配对的数目，匹配关系也存储在vpMatches12对象中了
		int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, vector<MapPoint *> &vpMatches12, const cv::Mat &R12, const cv::Mat &t12, const float th);


		// 主要用在Loop Closing的ComputeSim3
		// 在vpPoints集合中找到KeyFrame中MapPoint新的匹配的MapPoint
		// vpPoints集合是pKF匹配上的KeyFrame和其Graph邻居所有的KeyFrames包含的所有的MapPoint的集合
		int SearchByProjection(KeyFrame* pKF, cv::Mat Scw, const vector<MapPoint*> &vpPoints, vector<MapPoint*> &vpMatched, int th);


		// 主要用在Loop Closing的SearchAndFuse中
		// 在vpPoints集合中找到KeyFrame中MapPoint新的匹配的MapPoint，进行Fuse
		// vpPoints集合是pKF匹配上的KeyFrame和其Graph邻居所有的KeyFrames包含的所有的MapPoint的集合
		int Fuse(KeyFrame* pKF, cv::Mat Scw, const vector<MapPoint*> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint);
		

		/*
		// Matching for the Map Initialization (only used in the monocular case)
		int SearchForInitialization(Frame &F1, Frame &F2, vector<cv::Point2f> &vbPrevMatched, vector<int> &vnMatches12, int windowSize = 10);
		*/
		

	public:

		static const int TH_LOW;
		static const int TH_HIGH;
		static const int HISTO_LENGTH;

		protected:
			
			// 计算直方图中最大的三个的index
			void ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

			// 根据传入的进行阈值分割
			float RadiusByViewingCos(const float &viewCos);

			//
			bool CheckDistEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &F12, const KeyFrame *pKF);

			// 找到的匹配点应该是具有区分性的点，那么对存在的KeyPoint来说，在新的Frame找到对应的KeyPoint
			// 一方面找到的这个KeyPoint和原始的KeyPoint的汉明距离要小于一定的阈值（TH_LOW）,可以找到汉明距离最小的一个当做匹配KeyPoint
			// 另一方面找到的这个汉明距离最小的KeyPoint，要有区分度，也就是和汉明距离第二小的KeyPoint的汉明距离要有一定的差距
			// 下面变量就是为了说明这个区分度，最小的距离要比mfNNratio*第二小的距离小，这个KeyPoint才算真正满足条件
			// 值越大说明区分度越小，限制相对越小
			float mfNNratio;

			// 按道理说在一个图片上求得的KeyPoint和上一个图片上匹配的KeyPoint
			// 任意一对匹配的KeyPoint相对的角度应该是一样的，大概是图片变化的相对角度
			// 下面变量就是为了检测是不是最后找到的匹配点，有没有不符合这个一致性规律的
			// 不符合就从最终结果中去除
			bool mbCheckOrientation;

	};
}
#endif