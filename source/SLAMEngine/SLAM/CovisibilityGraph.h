// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.

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

#ifndef _COVISIBILITY_GRAPH_H
#define _COVISIBILITY_GRAPH_H
#include "KeyFrame.h"
#include "MapPoint.h"
#include "GraphNode.h"
#include <vector>
#include <set>
#include <mutex>

namespace SLAMRecon {
	
	class CovisibilityGraph {

	public:
		CovisibilityGraph();
		~CovisibilityGraph();
		
		// Update connections for incomming keyframe.
		void UpdateConnections(KeyFrame* pKF);
		
		// Get the top N connected keyframes based on weight for incomming keyframe.
		std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(KeyFrame* pKF, const int &N);
		
		// Get all connected keyframes for incomming keyframe.
		std::set<KeyFrame*> GetConnectedKeyFrames(KeyFrame* pKF);

		// Get all connected keyframes sorted by weight for incomming keyframe.
		std::vector<KeyFrame*> GetVectorCovisibleKeyFrames(KeyFrame* pKF);
		 
		// Get connected keyframes which weight is larger than incomming w for incomming keyframe.
		std::vector<KeyFrame*> GetCovisiblesByWeight(KeyFrame* pKF, const int &w);
		 
		void ClearConnections(KeyFrame* pKF);
		 
		int GetWeight(KeyFrame* pKF1, KeyFrame* pKF2);

		void clear();

	private: 
		GraphNode* getGraphNodeByKF(KeyFrame* pKF);

	private:
		// ALL nodes in this Covisivility Graph.
		std::map<KeyFrame*, GraphNode*> m_ALLVertexNodes;
		mutex m_MutexVertexNodes;
	};
} // namespace SLAMRecon
	
#endif // COVISIBILITYGRAPH_H