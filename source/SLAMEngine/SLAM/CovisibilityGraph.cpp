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

#include "CovisibilityGraph.h"

using namespace std;

namespace SLAMRecon {


	CovisibilityGraph::CovisibilityGraph() {

	}

	CovisibilityGraph::~CovisibilityGraph() {
		for (map<KeyFrame*, GraphNode*>::iterator mit = m_ALLVertexNodes.begin(), mend = m_ALLVertexNodes.end(); mit != mend; mit++) 			
			delete (*mit).second;
	}

	GraphNode* CovisibilityGraph::getGraphNodeByKF(KeyFrame* pKF) {
		unique_lock<mutex> lock(m_MutexVertexNodes);
		if (m_ALLVertexNodes.count(pKF))
			return m_ALLVertexNodes[pKF];
		else {
			GraphNode *pGN = new GraphNode(pKF);
			m_ALLVertexNodes[pKF] = pGN;
			return pGN;
		}
	}

	void CovisibilityGraph::UpdateConnections(KeyFrame* pKF) {
		
		vector<MapPoint*> vpMP;
		{ 
			vpMP = pKF->GetMapPointMatches(); 
		}
		 
		map<KeyFrame*, int> KFcounter;
		for (vector<MapPoint*>::iterator vit = vpMP.begin(), vend = vpMP.end(); vit != vend; vit++) {
			MapPoint* pMP = *vit;
	 
			if (!pMP)
				continue;
			if (pMP->isBad())
				continue;
			 
			map<KeyFrame*, size_t> observations = pMP->GetObservations();
			
			for (map<KeyFrame*, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
				if (mit->first->m_nKFId == pKF->m_nKFId)
					continue;
				KFcounter[mit->first]++;
			}
		}
		 
		if (KFcounter.empty())
			return;
	 
		int th = 15;
		int nth = 0;
		
		int nmax = 0;
		KeyFrame* pKFmax = NULL;
		
		vector<pair<int, KeyFrame*> > vPairs;
		vPairs.reserve(KFcounter.size());
	
	
		for (map<KeyFrame*, int>::iterator mit = KFcounter.begin(), mend = KFcounter.end(); mit != mend; mit++) {
			if (mit->second > nmax) {
				nmax = mit->second;
				pKFmax = mit->first;
			}
			if (mit->second >= th) { 
				nth++;
				getGraphNodeByKF(mit->first)->AddConnection(pKF, mit->second);
			}
		}
	
		if (nth == 0) 
			getGraphNodeByKF(pKFmax)->AddConnection(pKF, nmax); 
		 
		GraphNode *pVN = getGraphNodeByKF(pKF); 
		pVN->SetConnections(KFcounter);
	
	}

	vector<KeyFrame*> CovisibilityGraph::GetBestCovisibilityKeyFrames(KeyFrame* pKF, const int &N) {
		return  getGraphNodeByKF(pKF)->GetBestCovisibilityKeyFrames(N);
	}

	set<KeyFrame *> CovisibilityGraph::GetConnectedKeyFrames(KeyFrame* pKF) {
		return getGraphNodeByKF(pKF)->GetConnectedKeyFrames();
	}

	vector<KeyFrame* > CovisibilityGraph::GetVectorCovisibleKeyFrames(KeyFrame* pKF) {
		return getGraphNodeByKF(pKF)->GetVectorCovisibleKeyFrames();
	}

	std::vector<KeyFrame*> CovisibilityGraph::GetCovisiblesByWeight(KeyFrame* pKF, const int &w) {
		return getGraphNodeByKF(pKF)->GetCovisiblesByWeight(w);
	}

	void CovisibilityGraph::ClearConnections(KeyFrame* pKF) {
		

		GraphNode *pGN = getGraphNodeByKF(pKF);
		set<KeyFrame*> connectedKeyFrames = pGN->GetConnectedKeyFrames();  

		for (set<KeyFrame*>::iterator mit = connectedKeyFrames.begin(), mend = connectedKeyFrames.end(); mit != mend; mit++)
			getGraphNodeByKF(pKF)->EraseConnection(pKF);
		
		const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
		 
		for (size_t i = 0; i < vpMapPoints.size(); i++)
			if (vpMapPoints[i])
				vpMapPoints[i]->EraseObservation(pKF);
		 
		pGN->clear();
	}

	int CovisibilityGraph::GetWeight(KeyFrame* pKF1, KeyFrame* pKF2) {
		return getGraphNodeByKF(pKF1)->GetWeight(pKF2);
	}

	void CovisibilityGraph::clear() {
		unique_lock<mutex> lock(m_MutexVertexNodes);
		for (map<KeyFrame*, GraphNode*>::iterator mit = m_ALLVertexNodes.begin(), mend = m_ALLVertexNodes.end(); mit != mend; mit++) {
			(*mit).second->clear();
			delete (*mit).second;
		}
		m_ALLVertexNodes.clear();
	}
} // namespace SLAMRecon