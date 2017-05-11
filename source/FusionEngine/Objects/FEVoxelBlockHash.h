// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM
#ifndef _FE_VOXELBLOCKHASHING_H
#define _FE_VOXELBLOCKHASHING_H

#include <stdlib.h>
#include "../Utils/FELibDefines.h"

namespace FE
{
		/** \brief
		This is the central class for the voxel block hash
		implementation. It contains all the data needed on the CPU
		and a pointer to the data structure on the GPU.
		*/
		class FEVoxelBlockHash
		{
		public:
			typedef FEHashEntry IndexData;

			struct IndexCache {
				Vector3i blockPos;
				int blockPtr;
				_CPU_AND_GPU_CODE_ IndexCache(void) : blockPos(0x7fffffff), blockPtr(-1) {}
			};

			/** Maximum number of total entries. */
			static const CONSTPTR(int) noTotalEntries = SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE;
			static const CONSTPTR(int) voxelBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

		private:
			int lastFreeExcessListId;

			/** The actual data in the hash table. */
			Basis::MemoryBlock<FEHashEntry> *hashEntries;

			/** Identifies which entries of the overflow
			list are allocated. This is used if too
			many hash collisions caused the buckets to
			overflow.
			*/
			Basis::MemoryBlock<int> *excessAllocationList;
        
			MemoryDeviceType memoryType;

		public:
			FEVoxelBlockHash(MemoryDeviceType memoryType)
			{
				this->memoryType = memoryType;
				hashEntries = new Basis::MemoryBlock<FEHashEntry>(noTotalEntries, memoryType);
				excessAllocationList = new Basis::MemoryBlock<int>(SDF_EXCESS_LIST_SIZE, memoryType);
			}

			~FEVoxelBlockHash(void)
			{
				delete hashEntries;
				delete excessAllocationList;
			}

			/** Get the list of actual entries in the hash table. */
			const FEHashEntry *GetEntries(void) const { return hashEntries->GetData(memoryType); }
			FEHashEntry *GetEntries(void) { return hashEntries->GetData(memoryType); }

			const IndexData *getIndexData(void) const { return hashEntries->GetData(memoryType); }
			IndexData *getIndexData(void) { return hashEntries->GetData(memoryType); }

			/** Get the list that identifies which entries of the
			overflow list are allocated. This is used if too
			many hash collisions caused the buckets to overflow.
			*/
			const int *GetExcessAllocationList(void) const { return excessAllocationList->GetData(memoryType); }
			int *GetExcessAllocationList(void) { return excessAllocationList->GetData(memoryType); }

			int GetLastFreeExcessListId(void) { return lastFreeExcessListId; }
			void SetLastFreeExcessListId(int lastFreeExcessListId) { this->lastFreeExcessListId = lastFreeExcessListId; }

			/** Maximum number of total entries. */
			int getNumAllocatedVoxelBlocks(void) { return SDF_LOCAL_BLOCK_NUM; }
			int getVoxelBlockSize(void) { return SDF_BLOCK_SIZE3; }

			// Suppress the default copy constructor and assignment operator
			FEVoxelBlockHash(const FEVoxelBlockHash&);
			FEVoxelBlockHash& operator=(const FEVoxelBlockHash&);
		};
	}

#endif //_FE_VOXELBLOCKHASHING_H
