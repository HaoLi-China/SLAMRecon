// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.

#ifndef _GRAPH_NODE_H
#define _GRAPH_NODE_H

#include "KeyFrame.h"
#include <vector>
#include <set>
#include <mutex>

namespace SLAMRecon {

	class GraphNode {

	public:

		GraphNode(KeyFrame* pKF);
		~GraphNode();
		 
		// Add connection between this node and incoming keyframe.
		void AddConnection(KeyFrame* pKF, const int &weight);
		
		// Delete connection between this node and incoming keyframe.
		void EraseConnection(KeyFrame* pKF);
		
		// Set connections of this node.
		void SetConnections(std::map<KeyFrame*, int> ConnectedKFWs);
		
		// Sort for the connections based on the weight.
		void UpdateBestCovisibles();
		 
		// Get the top N connected keyframes based on weight.
		std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
		
		// Get all connected keyframes.
		std::set<KeyFrame *> GetConnectedKeyFrames();
		
		// Get all connected keyframes sorted by weight.
		std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();
		 
		// Get connected keyframes which weight is larger than incomming w.
		std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);

		static bool weightComp(int a, int b){
			return a > b;
		}
		
		int GetWeight(KeyFrame* pKF);
		 
		void clear();

	private:
		KeyFrame* m_pKeyFrame;  
		
		std::map<KeyFrame*, int> m_ConnectedKeyFrameWeights; 
		std::vector<KeyFrame*> m_vpOrderedConnectedKeyFrames; 
		std::vector<int> m_vOrderedWeights; 

		std::mutex m_MutexConnections;
	};
} // namespace SLAMRecon

#endif // GRAPHNODE_H