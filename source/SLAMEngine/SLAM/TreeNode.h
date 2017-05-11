// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.

#ifndef _TREE_NODE_H
#define _TREE_NODE_H

#include "KeyFrame.h"
#include <vector>
#include <set>
#include <mutex>

namespace SLAMRecon {

	class TreeNode {
	public:
		TreeNode(KeyFrame* pKF);
		~TreeNode();

		// Add/Erase Child and Set Parent for the node.
		void AddChild(KeyFrame* pKF);
		void EraseChild(KeyFrame* pKF);
		void SetParent(KeyFrame *pKF);

		// Returns the childs and parent node for the node.
		std::set<KeyFrame*> GetChilds();
		KeyFrame* GetParent();

		//Check incomming keyframe is the child node of the node or not.
		bool hasChild(KeyFrame *pKF);

		// Add/Get loop edge.
		void AddLoopEdge(KeyFrame* pKF);
		std::set<KeyFrame*> GetLoopEdges();

		bool m_bFirstConnection;  

		void clear();

	private:
		KeyFrame* m_pKeyFrame;  

		// Parent node of the node.
		KeyFrame* m_pParent; 

		// Children nodes of the node.
		std::set<KeyFrame*> m_spChildrens;  

		// Loop Edges of the node.
		std::set<KeyFrame*> m_spLoopEdges;  

		std::mutex m_MutexConnections;
	};
} // namespace SLAMRecon

#endif // TREENODE_H