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

#ifndef _KEY_FRAME_DATABASE_H_
#define _KEY_FRAME_DATABASE_H_

#include <vector>
#include <list>
#include "../ORB/ORBVocabulary.h"
#include "CovisibilityGraph.h"
#include "KeyFrame.h"
#include "Frame.h"

#include<mutex>

namespace SLAMRecon{

	class KeyFrameDatabase{
	public:
		KeyFrameDatabase(const ORBVocabulary* voc, CovisibilityGraph* cograph);
		~KeyFrameDatabase();
		 
		void add(KeyFrame* pKF);
		 
		void erase(KeyFrame* pKF);
		 
		void clear();
		
		// Relocalization
		std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F);
		
		// Loop Detection
		std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame* pKF, float minScore);

	protected:
		
		// Associated vocabulary
		const ORBVocabulary* m_pVoc;
		 
		CovisibilityGraph* m_pCoGraph;
		
		// Inverted file
		std::vector<list<KeyFrame*> > m_vInvertedFile;
		 
		std::mutex m_DBMutex;
	};

} // namespace SLAMRecon

#endif // KERFRAMEDATABASE_H