// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM
#ifndef _FE_LOCALVBA_H
#define _FE_LOCALVBA_H

#include <stdlib.h>
#include "../Utils/FELibDefines.h"

namespace FE
{
	/** \brief
		Stores the actual voxel content that is referred to by a
		FE::FEHashTable.
		*/
		template<class TVoxel>
		class FELocalVBA
		{
		private:
			Basis::MemoryBlock<TVoxel> *voxelBlocks;
			Basis::MemoryBlock<int> *allocationList;

			MemoryDeviceType memoryType;

		public:
			inline TVoxel *GetVoxelBlocks(void) { return voxelBlocks->GetData(memoryType); }
			inline const TVoxel *GetVoxelBlocks(void) const { return voxelBlocks->GetData(memoryType); }
			int *GetAllocationList(void) { return allocationList->GetData(memoryType); }

			int lastFreeBlockId;

			int allocatedSize;

			FELocalVBA(MemoryDeviceType memoryType, int noBlocks, int blockSize)
			{
				this->memoryType = memoryType;

				allocatedSize = noBlocks * blockSize;

				voxelBlocks = new Basis::MemoryBlock<TVoxel>(allocatedSize, memoryType);
				allocationList = new Basis::MemoryBlock<int>(noBlocks, memoryType);
			}

			~FELocalVBA(void)
			{
				delete voxelBlocks;
				delete allocationList;
			}

			// Suppress the default copy constructor and assignment operator
			FELocalVBA(const FELocalVBA&);
			FELocalVBA& operator=(const FELocalVBA&);
		};
	}

#endif //_FE_LOCALVBA_H