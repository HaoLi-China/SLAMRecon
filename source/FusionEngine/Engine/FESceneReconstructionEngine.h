// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM.
// Modified by authors of SLAMRecon.
#ifndef _FE_SCENERECONSTRUCTIONENGINE_H
#define _FE_SCENERECONSTRUCTIONENGINE_H

#include <math.h>

#include "../Utils/FELibDefines.h"

#include "../Objects/FEScene.h"
#include "../Objects/FEView.h"
#include "../Objects/FETrackingState.h"
#include "../Objects/FERenderState.h"

namespace FE
{
	/** \brief
		Interface to engines implementing the main KinectFusion
		depth integration process.

		These classes basically manage
		an FE::FEScene and fuse new image information
		into them.
		*/
	template<class TVoxel, class TIndex>
	class FESceneReconstructionEngine
	{
	public:
		/** Clear and reset a scene to set up a new empty
			one.
			*/
		virtual void ResetScene(FEScene<TVoxel, TIndex> *scene) = 0;

		/** Given a view with a new depth image, compute the
			visible blocks, allocate them and update the hash
			table so that the new image data can be integrated.
			*/
		virtual void AllocateSceneFromDepth(FEScene<TVoxel, TIndex> *scene, const FEView *view, const FETrackingState *trackingState,
			const FERenderState *renderState, bool onlyUpdateVisibleList = false) = 0;

		/** Update the voxel blocks by integrating depth and
			possibly colour information from the given view.
			*/
		virtual void IntegrateIntoScene(FEScene<TVoxel, TIndex> *scene, const FEView *view, const FETrackingState *trackingState,
			const FERenderState *renderState) = 0;

		virtual void AllocateSceneFromDepth(FEScene<TVoxel, TIndex> *scene, const FEView *view, const Matrix4f &M_d,
			const FERenderState *renderState, bool onlyUpdateVisibleList = false) = 0;

		virtual void AllocateSceneFromDepth(FEScene<TVoxel, FEVoxelBlockHash> *scene, const FEView *view, const int frameIndex, 
			const Matrix4f &M_d, const FERenderState *renderState, bool onlyUpdateVisibleList = false) = 0;

		virtual void IntegrateIntoScene(FEScene<TVoxel, TIndex> *scene, const FEView *view, const Matrix4f &M_d,
			const FERenderState *renderState) = 0;

		virtual void UpdateVisibleEntryIDsByBVLB(FEScene<TVoxel, FEVoxelBlockHash> *scene, const int index, const FERenderState *renderState) = 0;

		virtual void RepealFromScene(FEScene<TVoxel, FEVoxelBlockHash> *scene, const FEView *view, const Matrix4f &M_d,
			const FERenderState *renderState) = 0;

		FESceneReconstructionEngine(void) { }
		virtual ~FESceneReconstructionEngine(void) { }
	};
}
#endif //_FE_SCENERECONSTRUCTIONENGINE_H
