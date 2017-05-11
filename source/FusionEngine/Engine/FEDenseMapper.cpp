// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM
// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#include "FEDenseMapper.h"
#include "../Objects/FERenderState_VH.h"
#include "CUDA/FESceneReconstructionEngine_CUDA.h"

using namespace FE;

template<class TVoxel, class TIndex>
FEDenseMapper<TVoxel, TIndex>::FEDenseMapper(const FELibSettings *settings)
{
	switch (settings->deviceType)
	{
	case FELibSettings::DEVICE_CUDA:
		sceneRecoEngine = new FESceneReconstructionEngine_CUDA<TVoxel,TIndex>();
		break;
	}
}

template<class TVoxel, class TIndex>
FEDenseMapper<TVoxel,TIndex>::~FEDenseMapper()
{
	delete sceneRecoEngine;
}

template<class TVoxel, class TIndex>
void FEDenseMapper<TVoxel,TIndex>::ResetScene(FEScene<TVoxel,TIndex> *scene)
{
	sceneRecoEngine->ResetScene(scene);
}

template<class TVoxel, class TIndex>
void FEDenseMapper<TVoxel,TIndex>::ProcessFrame(const FEView *view, const FETrackingState *trackingState, FEScene<TVoxel,TIndex> *scene, FERenderState *renderState)
{
	// allocation
	sceneRecoEngine->AllocateSceneFromDepth(scene, view, trackingState, renderState);

	// integration
	sceneRecoEngine->IntegrateIntoScene(scene, view, trackingState, renderState);
}

template<class TVoxel, class TIndex>
void FEDenseMapper<TVoxel, TIndex>::ProcessFrame(const FEView *view, const Matrix4f &M_d, FEScene<TVoxel, TIndex> *scene, FERenderState *renderState)
{
	// allocation
	sceneRecoEngine->AllocateSceneFromDepth(scene, view, M_d, renderState);

	// integration
	sceneRecoEngine->IntegrateIntoScene(scene, view, M_d, renderState);
}

template<class TVoxel, class TIndex>
void FEDenseMapper<TVoxel, TIndex>::ProcessFrame(const FEView *view, const int index, const Matrix4f &M_d, FEScene<TVoxel, TIndex> *scene, FERenderState *renderState)
{
	// allocation
	sceneRecoEngine->AllocateSceneFromDepth(scene, view, index, M_d, renderState);

	// integration
	sceneRecoEngine->IntegrateIntoScene(scene, view, M_d, renderState);
}

template<class TVoxel, class TIndex>
void FEDenseMapper<TVoxel, TIndex>::Reintegration(const FEView *view, const int frameIndex, const Matrix4f &old_M, const Matrix4f &new_M, FEScene<TVoxel, TIndex> *scene, FERenderState *renderState)
{
	// allocation
	//sceneRecoEngine->AllocateSceneFromDepth(scene, view, old_M, renderState);

	// update visibleEntry IDs
	sceneRecoEngine->UpdateVisibleEntryIDsByBVLB(scene, frameIndex, renderState);

	// repeal
	sceneRecoEngine->RepealFromScene(scene, view, old_M, renderState);

	// allocation
	sceneRecoEngine->AllocateSceneFromDepth(scene, view, frameIndex, new_M, renderState);

	// integration
	sceneRecoEngine->IntegrateIntoScene(scene, view, new_M, renderState);
}

template<class TVoxel, class TIndex>
void FEDenseMapper<TVoxel,TIndex>::UpdateVisibleList(const FEView *view, const FETrackingState *trackingState, FEScene<TVoxel,TIndex> *scene, FERenderState *renderState)
{
	sceneRecoEngine->AllocateSceneFromDepth(scene, view, trackingState, renderState, true);
}

template class FE::FEDenseMapper<FEVoxel, FEVoxelIndex>;
