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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H
#include "Map.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "CovisibilityGraph.h"
#include "SpanningTree.h"
#include "LoopClosing.h"
#include <g2o/types/sim3/sim3.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>
namespace SLAMRecon {

	class Optimizer {

	public:
		static int PoseOptimization(Frame* pFrame);

		static void LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap, CovisibilityGraph* pCoGraph);

		static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, g2o::Sim3 &g2oS12, const float th2);

		void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF, const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
			const LoopClosing::KeyFrameAndPose &CorrectedSim3, const map<KeyFrame *, set<KeyFrame *> > &LoopConnections, CovisibilityGraph* pCoGraph, SpanningTree* pSpanTree);

		void static GlobalBundleAdjustemnt(Map* pMap, int nIterations = 5, bool *pbStopFlag = NULL,
			const unsigned long nLoopKF = 0, const bool bRobust = true);

		void static BundleAdjustment(Map* pMap, const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
			int nIterations = 5, bool *pbStopFlag = NULL, const unsigned long nLoopKF = 0,
			const bool bRobust = true);

	};
} // namespace SLAMRecon

#endif // OPTIMIZER_H