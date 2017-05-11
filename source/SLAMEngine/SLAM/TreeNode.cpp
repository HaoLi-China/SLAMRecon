// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.

#include "TreeNode.h"

using namespace std;

namespace SLAMRecon{


	TreeNode::TreeNode(KeyFrame* pKF)
		:m_pKeyFrame(pKF), m_pParent(NULL), m_bFirstConnection(true)
	{

	}

	TreeNode::~TreeNode() {
	}

	void TreeNode::AddChild(KeyFrame *pKF) {
		unique_lock<mutex> lockCon(m_MutexConnections);
		m_spChildrens.insert(pKF);
	}

	void TreeNode::EraseChild(KeyFrame *pKF) {
		unique_lock<mutex> lockCon(m_MutexConnections);
		m_spChildrens.erase(pKF);
	}

	void TreeNode::SetParent(KeyFrame *pKF) {
		unique_lock<mutex> lockCon(m_MutexConnections);
		m_pParent = pKF;
	}

	set<KeyFrame*> TreeNode::GetChilds() {
		unique_lock<mutex> lockCon(m_MutexConnections);
		return m_spChildrens;
	}
	
	bool TreeNode::hasChild(KeyFrame *pKF) {
		unique_lock<mutex> lockCon(m_MutexConnections);
		return m_spChildrens.count(pKF);
	}

	KeyFrame* TreeNode::GetParent() {
		unique_lock<mutex> lockCon(m_MutexConnections);
		return m_pParent;
	}

	void TreeNode::AddLoopEdge(KeyFrame *pKF) {
		unique_lock<mutex> lockCon(m_MutexConnections);
		m_spLoopEdges.insert(pKF);
	}

	set<KeyFrame*> TreeNode::GetLoopEdges() {
		unique_lock<mutex> lockCon(m_MutexConnections);
		return m_spLoopEdges;
	}

	void  TreeNode::clear() {
		unique_lock<mutex> lockCon(m_MutexConnections);
		m_spChildrens.clear();
		m_spLoopEdges.clear();
	}
} // namespace SLAMRecon