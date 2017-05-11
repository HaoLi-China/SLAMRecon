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

#include "SpanningTree.h"

using namespace std;

namespace SLAMRecon {
	SpanningTree::SpanningTree(CovisibilityGraph *pCVG)
		:m_pCovisibilityGraph(pCVG)
	{

	}

	SpanningTree::~SpanningTree(){
		for (map<KeyFrame*, TreeNode*>::iterator mit = m_ALLVertexNodes.begin(), mend = m_ALLVertexNodes.end(); mit != mend; mit++) 
			delete (*mit).second;
	}

	TreeNode* SpanningTree::getTreeNodeByKF(KeyFrame* pKF) {
		unique_lock<mutex> lock(m_MutexVertexNodes);
		if (m_ALLVertexNodes.count(pKF))
			return m_ALLVertexNodes[pKF];
		else {
			TreeNode *pTN = new TreeNode(pKF);
			m_ALLVertexNodes[pKF] = pTN;
			return pTN;
		}
	}

	void SpanningTree::UpdateConnections(KeyFrame* pKF) { 
		if (pKF->m_nKFId != 0 && getTreeNodeByKF(pKF)->m_bFirstConnection) {
			getTreeNodeByKF(pKF)->m_bFirstConnection = false;
			getTreeNodeByKF(m_pCovisibilityGraph->GetBestCovisibilityKeyFrames(pKF, 1).front())->AddChild(pKF);
			getTreeNodeByKF(pKF)->SetParent(m_pCovisibilityGraph->GetBestCovisibilityKeyFrames(pKF, 1).front());
		}
		
	}

	void SpanningTree::ChangeParent(KeyFrame *pChildKF, KeyFrame *pParentKF) {
			getTreeNodeByKF(pParentKF)->AddChild(pChildKF);
			getTreeNodeByKF(pChildKF)->SetParent(pParentKF);
	}

	set<KeyFrame*> SpanningTree::GetChilds(KeyFrame* pKF) {
		return getTreeNodeByKF(pKF)->GetChilds();
	}

	KeyFrame* SpanningTree::GetParent(KeyFrame* pKF) {
		return getTreeNodeByKF(pKF)->GetParent();
	}

	bool SpanningTree::hasChild(KeyFrame *pChildKF, KeyFrame *pParentKF) {
		return getTreeNodeByKF(pParentKF)->hasChild(pChildKF);
	}

	void SpanningTree::ClearConnections(KeyFrame* pKF) {
		TreeNode* pTN = getTreeNodeByKF(pKF);
		 
		set<KeyFrame*> sParentCandidates;
		sParentCandidates.insert(pTN->GetParent());
		
		// Assign at each iteration one children with a parent (the pair with highest covisibility weight)
		// Include that children as new parent candidate for the rest
		set<KeyFrame*> spChildrens = pTN->GetChilds();
		while (!spChildrens.empty()) {

			bool bContinue = false;
		
			int max = -1;
			KeyFrame* pC;
			KeyFrame* pP;
		 
			for (set<KeyFrame*>::iterator sit = spChildrens.begin(), send = spChildrens.end(); sit != send; sit++) {
				KeyFrame* pKF2 = *sit;

				if (pKF->isBad())
					continue;
		
				vector<KeyFrame*> vpConnected = m_pCovisibilityGraph->GetVectorCovisibleKeyFrames(pKF2);

				for (size_t i = 0, iend = vpConnected.size(); i < iend; i++) {

					for (set<KeyFrame*>::iterator spcit = sParentCandidates.begin(), spcend = sParentCandidates.end(); spcit != spcend; spcit++) {

						if (vpConnected[i]->m_nKFId == (*spcit)->m_nKFId) {
							int w = m_pCovisibilityGraph->GetWeight(pKF2, vpConnected[i]);
							if (w > max) {
								pC = pKF2;
								pP = vpConnected[i];
								max = w;
								bContinue = true;
							}
						}
					}
				}
			}
			 
			if (bContinue) {
				ChangeParent(pC, pP);
				sParentCandidates.insert(pC);  
				spChildrens.erase(pC);  
			} else
				break;  
		}
		 
		if (!spChildrens.empty())
			for (set<KeyFrame*>::iterator sit = spChildrens.begin(); sit != spChildrens.end(); sit++) 
				ChangeParent(*sit, pTN->GetParent()); 
		 
		 
		getTreeNodeByKF(pTN->GetParent())->EraseChild(pKF);
		 
		pKF->SetBad(pTN->GetParent());
		
	}

	void SpanningTree::AddLoopEdge(KeyFrame* pKF1, KeyFrame* pKF2) {
		getTreeNodeByKF(pKF1)->AddLoopEdge(pKF2);
		getTreeNodeByKF(pKF2)->AddLoopEdge(pKF1);
	}

	std::set<KeyFrame*> SpanningTree::GetLoopEdges(KeyFrame* pKF) {
		return getTreeNodeByKF(pKF)->GetLoopEdges();
	}

	void SpanningTree::clear() {
		unique_lock<mutex> lock(m_MutexVertexNodes);
		for (map<KeyFrame*, TreeNode*>::iterator mit = m_ALLVertexNodes.begin(), mend = m_ALLVertexNodes.end(); mit != mend; mit++) {
			// delete node
			(*mit).second->clear();
			delete (*mit).second;
		}
		m_ALLVertexNodes.clear();
	}

} // namespace SLAMRecon