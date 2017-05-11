// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM.
// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#ifndef _FE_SCENERECONSTRUCTIONENGINE_CUDA_H
#define _FE_SCENERECONSTRUCTIONENGINE_CUDA_H

#include "../FESceneReconstructionEngine.h"

namespace FE
{
	template<class TVoxel, class TIndex>
	class FESceneReconstructionEngine_CUDA : public FESceneReconstructionEngine < TVoxel, TIndex >
	{};

	template<class TVoxel>
	class FESceneReconstructionEngine_CUDA<TVoxel, FEVoxelBlockHash> : public FESceneReconstructionEngine < TVoxel, FEVoxelBlockHash >
	{
	private:
		void *allocationTempData_device;
		void *allocationTempData_host;
		unsigned char *entriesAllocType_device;
		Vector4s *blockCoords_device;

		unsigned char *temStructure_device;//temperal structure for building binary visible table

	public:
		void ResetScene(FEScene<TVoxel, FEVoxelBlockHash> *scene);

		void AllocateSceneFromDepth(FEScene<TVoxel, FEVoxelBlockHash> *scene, const FEView *view, const FETrackingState *trackingState,
			const FERenderState *renderState, bool onlyUpdateVisibleList = false);

		void IntegrateIntoScene(FEScene<TVoxel, FEVoxelBlockHash> *scene, const FEView *view, const FETrackingState *trackingState,
			const FERenderState *renderState);

		void AllocateSceneFromDepth(FEScene<TVoxel, FEVoxelBlockHash> *scene, const FEView *view, const Matrix4f &M_d,
			const FERenderState *renderState, bool onlyUpdateVisibleList = false);

		void AllocateSceneFromDepth(FEScene<TVoxel, FEVoxelBlockHash> *scene, const FEView *view, const int frameIndex, const Matrix4f &M_d,
			const FERenderState *renderState, bool onlyUpdateVisibleList = false);

		void IntegrateIntoScene(FEScene<TVoxel, FEVoxelBlockHash> *scene, const FEView *view, const Matrix4f &M_d,
			const FERenderState *renderState);

		void UpdateVisibleEntryIDsByBVLB(FEScene<TVoxel, FEVoxelBlockHash> *scene, const int index, const FERenderState *renderState);

		void RepealFromScene(FEScene<TVoxel, FEVoxelBlockHash> *scene, const FEView *view, const Matrix4f &M_d,
			const FERenderState *renderState);

		//void testVLB();
		
		FESceneReconstructionEngine_CUDA(void);
		~FESceneReconstructionEngine_CUDA(void);
	};
}
#endif //_FE_SCENERECONSTRUCTIONENGINE_CUDA_H
