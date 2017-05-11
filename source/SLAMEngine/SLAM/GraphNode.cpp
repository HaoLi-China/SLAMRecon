// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.

#include "GraphNode.h"

using namespace std;

namespace SLAMRecon{

	
	GraphNode::GraphNode(KeyFrame* pKF)
		:m_pKeyFrame(pKF)
	{
		
	}

	GraphNode::~GraphNode() {

	}
	
	void GraphNode::AddConnection(KeyFrame *pKF, const int &weight) {
		{
			unique_lock<mutex> lock(m_MutexConnections);
			if (!m_ConnectedKeyFrameWeights.count(pKF))
				m_ConnectedKeyFrameWeights[pKF] = weight; 
			else if (m_ConnectedKeyFrameWeights[pKF] != weight)
				m_ConnectedKeyFrameWeights[pKF] = weight;
		} 
		UpdateBestCovisibles();
	}
	
	void GraphNode::EraseConnection(KeyFrame* pKF) {
		bool bUpdate = false;
		{
			unique_lock<mutex> lock(m_MutexConnections);
			if (m_ConnectedKeyFrameWeights.count(pKF)) {
				m_ConnectedKeyFrameWeights.erase(pKF);
				bUpdate = true;
			}
		}
		
		if (bUpdate)
			UpdateBestCovisibles();
	}

	void GraphNode::SetConnections(map<KeyFrame*, int> ConnectedKFWs) {
		{
			unique_lock<mutex> lock(m_MutexConnections);
			m_ConnectedKeyFrameWeights = ConnectedKFWs;
		}
		UpdateBestCovisibles();
	}

	void GraphNode::UpdateBestCovisibles() {
		unique_lock<mutex> lock(m_MutexConnections);

		vector<pair<int, KeyFrame*> > vPairs;
		vPairs.reserve(m_ConnectedKeyFrameWeights.size());
		for (map<KeyFrame*, int>::iterator mit = m_ConnectedKeyFrameWeights.begin(), mend = m_ConnectedKeyFrameWeights.end(); mit != mend; mit++)
			vPairs.push_back(make_pair(mit->second, mit->first));

		sort(vPairs.begin(), vPairs.end());
		list<KeyFrame*> lKFs;
		list<int> lWs;
		for (size_t i = 0, iend = vPairs.size(); i < iend; i++) {
			lKFs.push_front(vPairs[i].second);
			lWs.push_front(vPairs[i].first);
		}

		m_vpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(), lKFs.end());
		m_vOrderedWeights = vector<int>(lWs.begin(), lWs.end());
	}

	vector<KeyFrame*> GraphNode::GetBestCovisibilityKeyFrames(const int &N) {
		unique_lock<mutex> lock(m_MutexConnections);
		if ((int)m_vpOrderedConnectedKeyFrames.size() < N)
			return m_vpOrderedConnectedKeyFrames;
		else
			return vector<KeyFrame*>(m_vpOrderedConnectedKeyFrames.begin(), m_vpOrderedConnectedKeyFrames.begin() + N);
	}

	set<KeyFrame*> GraphNode::GetConnectedKeyFrames() {
		unique_lock<mutex> lock(m_MutexConnections);
		set<KeyFrame*> s;
		for (map<KeyFrame*, int>::iterator mit = m_ConnectedKeyFrameWeights.begin(); mit != m_ConnectedKeyFrameWeights.end(); mit++)
			s.insert(mit->first);
		return s;
	}

	vector<KeyFrame*> GraphNode::GetVectorCovisibleKeyFrames() {
		unique_lock<mutex> lock(m_MutexConnections);
		return m_vpOrderedConnectedKeyFrames;
	}

	vector<KeyFrame*> GraphNode::GetCovisiblesByWeight(const int &w) {
		unique_lock<mutex> lock(m_MutexConnections);

		if (m_vpOrderedConnectedKeyFrames.empty())
			return vector<KeyFrame*>();

		vector<int>::iterator it = upper_bound(m_vOrderedWeights.begin(), m_vOrderedWeights.end(), w, weightComp);
		if (it == m_vOrderedWeights.end())
			return vector<KeyFrame*>();
		else {
			int n = it - m_vOrderedWeights.begin();
			return vector<KeyFrame*>(m_vpOrderedConnectedKeyFrames.begin(), m_vpOrderedConnectedKeyFrames.begin() + n);
		}
	}

	int GraphNode::GetWeight(KeyFrame *pKF) {
		unique_lock<mutex> lock(m_MutexConnections);
		if (m_ConnectedKeyFrameWeights.count(pKF))
			return m_ConnectedKeyFrameWeights[pKF];
		else
			return 0;
	}

	void GraphNode::clear() {
		unique_lock<mutex> lock(m_MutexConnections);
		m_ConnectedKeyFrameWeights.clear();
		m_vpOrderedConnectedKeyFrames.clear();
		m_vOrderedWeights.clear();
	}

} // namespace SLAMRecon