// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM
// Modified by authors of SLAMRecon.
#ifndef _FE_RENDERSTATE_VH_H
#define _FE_RENDERSTATE_VH_H

#include <stdlib.h>

#include "FERenderState.h"
#include "MemoryBlock.h"
#include "Define.h"

namespace FE
{
	/** \brief
		Stores the render state used by the SceneReconstruction
		and visualisation engines, as used by voxel hashing.
		*/
	class FERenderState_VH : public FERenderState
	{
	private:
		MemoryDeviceType memoryType;

		/** A list of "visible entries", that are currently
		being processed by the tracker.
		*/
		Basis::MemoryBlock<int> *visibleEntryIDs;

		/** A list of "visible entries", that are
		currently being processed by integration
		and tracker.
		*/
		Basis::MemoryBlock<uchar> *entriesVisibleType;

		/** A bit list of "visible entries", that are
		currently being processed by integration
		and tracker.
		*/
		Basis::MemoryBlock<uchar> *visibleList;

		/** A block of list of "visible entries", that are
		currently being processed by reintegration.
		*/
		BitVisibleListBlock *bvlb;

	public:
		/** Number of entries in the live list. */
		int noVisibleEntries;

		FERenderState_VH(int noTotalEntries, const Vector2i & imgSize, float vf_min, float vf_max, MemoryDeviceType memoryType = MEMORYDEVICE_CPU)
			: FERenderState(imgSize, vf_min, vf_max, memoryType)
		{
			this->memoryType = memoryType;

			visibleEntryIDs = new Basis::MemoryBlock<int>(SDF_LOCAL_BLOCK_NUM, memoryType);
			entriesVisibleType = new Basis::MemoryBlock<uchar>(noTotalEntries, memoryType);
			visibleList = new Basis::MemoryBlock<uchar>(noTotalEntries / 8 + 1, memoryType);

#ifdef USE_VISIBLELIST_BLOCK
			bvlb = new BitVisibleListBlock(noTotalEntries / 8 + 1, IMAGES_BLOCK_SIZE, MEMORYDEVICE_CPU);
#else
			bvlb = NULL;
#endif // USE_VISIBLELIST_BLOCK

			noVisibleEntries = 0;
		}

		~FERenderState_VH()
		{
			delete visibleEntryIDs;
			delete entriesVisibleType;
			delete visibleList;
			delete bvlb;
		}

		/** Get the list of "visible entries", that are currently
		processed by the tracker.
		*/
		const int *GetVisibleEntryIDs(void) const { return visibleEntryIDs->GetData(memoryType); }
		int *GetVisibleEntryIDs(void) { return visibleEntryIDs->GetData(memoryType); }

		/** Get the list of "visible entries", that are
		currently processed by integration and tracker.
		*/
		uchar *GetEntriesVisibleType(void) { return entriesVisibleType->GetData(memoryType); }

		/** A block of list of "visible entries".
		*/
		BitVisibleListBlock *GetVisibleListBlock(void) { return bvlb; }

		Basis::MemoryBlock<uchar> *CetVisibleList(void) { return visibleList; }
		uchar *CetVisibleListPtr(void) { return visibleList->GetData(memoryType); }
	};
}

#endif //_FE_RENDERSTATE_VH_H
