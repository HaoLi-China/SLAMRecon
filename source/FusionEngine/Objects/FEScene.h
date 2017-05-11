// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM
#ifndef _FE_SCENE_H
#define _FE_SCENE_H

#include "../Utils/FELibDefines.h"

#include "FESceneParams.h"
#include "FELocalVBA.h"

namespace FE
{
		/** \brief
		Represents the 3D world model as a hash of small voxel
		blocks
		*/
		template<class TVoxel, class TIndex>
		class FEScene
		{
		public:
			/** Scene parameters like voxel size etc. */
			const FESceneParams *sceneParams;

			/** Hash table to reference the 8x8x8 blocks */
			TIndex index;

			/** Current local content of the 8x8x8 voxel blocks -- stored host or device */
			FELocalVBA<TVoxel> localVBA;

			FEScene(const FESceneParams *sceneParams, MemoryDeviceType memoryType)
				: index(memoryType), localVBA(memoryType, index.getNumAllocatedVoxelBlocks(), index.getVoxelBlockSize())
			{
				this->sceneParams = sceneParams;
			}

			~FEScene(void)
			{
			}

			// Suppress the default copy constructor and assignment operator
			FEScene(const FEScene&);
			FEScene& operator=(const FEScene&);
		};
}
#endif