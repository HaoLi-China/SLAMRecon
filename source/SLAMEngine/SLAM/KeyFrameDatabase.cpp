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

#include "KeyFrameDatabase.h"

using namespace std;

namespace SLAMRecon{
	KeyFrameDatabase::KeyFrameDatabase(const ORBVocabulary* voc, CovisibilityGraph* cograph)
		:m_pVoc(voc), m_pCoGraph(cograph)
	{
		m_vInvertedFile.resize(voc->size());
	}

	KeyFrameDatabase::~KeyFrameDatabase(){
		
	}

	void KeyFrameDatabase::add(KeyFrame* pKF) {
		unique_lock<mutex> lock(m_DBMutex);
		
		for (DBoW2::BowVector::const_iterator vit = pKF->m_BowVec.begin(), vend = pKF->m_BowVec.end(); vit != vend; vit++)
			m_vInvertedFile[vit->first].push_back(pKF);
	}

	void KeyFrameDatabase::erase(KeyFrame* pKF) {
		unique_lock<mutex> lock(m_DBMutex); 
		for (DBoW2::BowVector::const_iterator vit = pKF->m_BowVec.begin(), vend = pKF->m_BowVec.end(); vit != vend; vit++) { 
			list<KeyFrame*> &lKFs = m_vInvertedFile[vit->first]; 
			for (list<KeyFrame*>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++) {
				if (pKF == *lit) {
					lKFs.erase(lit);
					break;
				}
			}
		}
	}

	void KeyFrameDatabase::clear() {
		m_vInvertedFile.clear();
		m_vInvertedFile.resize(m_pVoc->size());
	}

	vector<KeyFrame*> KeyFrameDatabase::DetectRelocalizationCandidates(Frame *F)
	{
		list<KeyFrame*> lKFsSharingWords; 
		{
			unique_lock<mutex> lock(m_DBMutex);

			for (DBoW2::BowVector::const_iterator vit = F->m_BowVec.begin(), vend = F->m_BowVec.end(); vit != vend; vit++) {

				list<KeyFrame*> &lKFs = m_vInvertedFile[vit->first];

				for (list<KeyFrame*>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++) {

					KeyFrame* pKFi = *lit; 
					if (pKFi->mnRelocQuery != F->m_nFId) {
						pKFi->mnRelocWords = 0;
						pKFi->mnRelocQuery = F->m_nFId;
						lKFsSharingWords.push_back(pKFi);
					}
					pKFi->mnRelocWords++;
				}
			}
		} 
		if (lKFsSharingWords.empty())
			return vector<KeyFrame*>();
		cout << "lKFsSharingWords.size() " << lKFsSharingWords.size() << endl;
		 
		int maxCommonWords = 0;
		for (list<KeyFrame*>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++) {
			if ((*lit)->mnRelocWords > maxCommonWords)
				maxCommonWords = (*lit)->mnRelocWords;
		}
		 
		int minCommonWords = maxCommonWords*0.8f;


		int nscores = 0;  
		list<pair<float, KeyFrame*> > lScoreAndMatch;  
		 
		for (list<KeyFrame*>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++) {

			KeyFrame* pKFi = *lit;

			if (pKFi->mnRelocWords > minCommonWords) {
				nscores++;
				float si = m_pVoc->score(F->m_BowVec, pKFi->m_BowVec); 
				pKFi->mRelocScore = si;  
				lScoreAndMatch.push_back(make_pair(si, pKFi));
			}
		}
		 
		if (lScoreAndMatch.empty())
			return vector<KeyFrame*>();
		cout << "lScoreAndMatch.size() " << lScoreAndMatch.size() << endl;
		 

		list<pair<float, KeyFrame*> > lAccScoreAndMatch;  
		float bestAccScore = 0;

		for (list<pair<float, KeyFrame*> >::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++) {
			KeyFrame* pKFi = it->second;
			 
			vector<KeyFrame*> vpNeighs = m_pCoGraph->GetBestCovisibilityKeyFrames(pKFi,10);

			float bestScore = it->first;
			float accScore = bestScore;
			KeyFrame* pBestKF = pKFi;
			for (vector<KeyFrame*>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++) {
				KeyFrame* pKF2 = *vit;

				if (pKF2->mnRelocQuery != F->m_nFId)
					continue;

				accScore += pKF2->mRelocScore;
				if (pKF2->mRelocScore > bestScore) {
					pBestKF = pKF2;
					bestScore = pKF2->mRelocScore;
				}
			}
			lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
			if (accScore > bestAccScore)
				bestAccScore = accScore;
		}
		 
		float minScoreToRetain = 0.75f*bestAccScore;

		set<KeyFrame*> spAlreadyAddedKF;  
		vector<KeyFrame*> vpRelocCandidates;  
		vpRelocCandidates.reserve(lAccScoreAndMatch.size());
		for (list<pair<float, KeyFrame*> >::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++) {
			const float &si = it->first;
			if (si > minScoreToRetain) {
				KeyFrame* pKFi = it->second;
				if (!spAlreadyAddedKF.count(pKFi)) {
					vpRelocCandidates.push_back(pKFi);
					spAlreadyAddedKF.insert(pKFi);
				}
			}
		}

		return vpRelocCandidates;
	}

	vector<KeyFrame*> KeyFrameDatabase::DetectLoopCandidates(KeyFrame* pKF, float minScore) {
		 
		set<KeyFrame*> spConnectedKeyFrames = m_pCoGraph->GetConnectedKeyFrames(pKF);
		list<KeyFrame*> lKFsSharingWords;
		{
			unique_lock<mutex> lock(m_DBMutex);

			for (DBoW2::BowVector::const_iterator vit = pKF->m_BowVec.begin(), vend = pKF->m_BowVec.end(); vit != vend; vit++) {

				list<KeyFrame*> &lKFs = m_vInvertedFile[vit->first];

				for (list<KeyFrame*>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++) {
					KeyFrame* pKFi = *lit;
					if (pKFi->m_nLoopQuery != pKF->m_nKFId) {
						pKFi->m_nLoopWords = 0;
						if (!spConnectedKeyFrames.count(pKFi)) {
							pKFi->m_nLoopQuery = pKF->m_nKFId;
							lKFsSharingWords.push_back(pKFi);
						}
					}
					pKFi->m_nLoopWords++;
				}
			}
		}
		 
		if (lKFsSharingWords.empty())
			return vector<KeyFrame*>();

		cout << "lKFsSharingWords.size() " << lKFsSharingWords.size() << endl;
		 
		int maxCommonWords = 0;
		for (list<KeyFrame*>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++) {
			if ((*lit)->m_nLoopWords > maxCommonWords)
				maxCommonWords = (*lit)->m_nLoopWords;
		} 
		int minCommonWords = maxCommonWords*0.8f;

		int nscores = 0;  
		list<pair<float, KeyFrame*> > lScoreAndMatch;  
		 
		for (list<KeyFrame*>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++) {
			KeyFrame* pKFi = *lit;

			if (pKFi->m_nLoopWords > minCommonWords) {
				nscores++;

				float si = m_pVoc->score(pKF->m_BowVec, pKFi->m_BowVec);  

				pKFi->m_LoopScore = si;   
				if (si >= minScore)  
					lScoreAndMatch.push_back(make_pair(si, pKFi));
			}
		}
		 
		if (lScoreAndMatch.empty())
			return vector<KeyFrame*>();

		cout << "lScoreAndMatch.size() " << lScoreAndMatch.size() << endl;
		  
		list<pair<float, KeyFrame*> > lAccScoreAndMatch;  
		float bestAccScore = minScore;  

		for (list<pair<float, KeyFrame*> >::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++) {
			KeyFrame* pKFi = it->second;
			 
			vector<KeyFrame*> vpNeighs = m_pCoGraph->GetBestCovisibilityKeyFrames(pKFi, 10);

			float bestScore = it->first;
			float accScore = it->first;
			KeyFrame* pBestKF = pKFi;
			for (vector<KeyFrame*>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++) {
				KeyFrame* pKF2 = *vit;
				if (pKF2->m_nLoopQuery == pKF->m_nKFId && pKF2->m_nLoopWords > minCommonWords) {
					accScore += pKF2->m_LoopScore;
					if (pKF2->m_LoopScore > bestScore) {
						pBestKF = pKF2;
						bestScore = pKF2->m_LoopScore;
					}
				}
			}

			lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
			if (accScore > bestAccScore)
				bestAccScore = accScore;
		}
		 
		float minScoreToRetain = 0.75f*bestAccScore;

		set<KeyFrame*> spAlreadyAddedKF;  
		vector<KeyFrame*> vpLoopCandidates;  
		vpLoopCandidates.reserve(lAccScoreAndMatch.size());

		for (list<pair<float, KeyFrame*> >::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++) {
			if (it->first > minScoreToRetain) {
				KeyFrame* pKFi = it->second;
				if (!spAlreadyAddedKF.count(pKFi)) {
					vpLoopCandidates.push_back(pKFi);
					spAlreadyAddedKF.insert(pKFi);
				}
			}
		}
		cout << "vpLoopCandidates.size() " << vpLoopCandidates.size() << endl;
		return vpLoopCandidates;
	}
} // namespace SLAMRecon
