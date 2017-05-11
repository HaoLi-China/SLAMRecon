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

#ifndef _SPANNING_TREE_H
#define _SPANNING_TREE_H

#include "KeyFrame.h"
#include "TreeNode.h"
#include "GraphNode.h"
#include "CovisibilityGraph.h"

#include <vector>
#include <set>
#include <mutex>

namespace SLAMRecon {

	class SpanningTree {

	public:
		SpanningTree(CovisibilityGraph *pCVG);
		~SpanningTree();
		 
		// Update connections for pKF.
		void SpanningTree::UpdateConnections(KeyFrame* pKF);
		
		// Change the parent of pChildKF to pParentKF.
		void ChangeParent(KeyFrame *pChildKF, KeyFrame *pParentKF);
		
		// Returns the childs and parent node for the pKF.
		set<KeyFrame*> GetChilds(KeyFrame* pKF);
		KeyFrame* GetParent(KeyFrame* pKF);
			
		// Check pParentKF is the parent node of pChildKF or not.
		bool hasChild(KeyFrame *pChildKF, KeyFrame *pParentKF);
		 
		void ClearConnections(KeyFrame* pKF);
		 
		// Add Loop edge between pKF1 and pKF2.
		void AddLoopEdge(KeyFrame* pKF1, KeyFrame* pKF2);
		std::set<KeyFrame*> GetLoopEdges(KeyFrame* pKF);

		void clear();

	private: 
		TreeNode* getTreeNodeByKF(KeyFrame* pKF);

	private:
		// All nodes in this Spanning Tree.
		std::map<KeyFrame*, TreeNode*> m_ALLVertexNodes;
		CovisibilityGraph *m_pCovisibilityGraph;

		std::mutex m_MutexVertexNodes;
	};

} // namespace SLAMRecon

#endif //SPANNINGTREE_H