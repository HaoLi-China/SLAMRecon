// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM.
// Modified by authors of SLAMRecon.
#ifndef _FE_VISUALISATIONENGINE_CUDA_H
#define _FE_VISUALISATIONENGINE_CUDA_H

#include "../FEVisualisationEngine.h"

struct RenderingBlock;

namespace FE
{
	template<class TVoxel, class TIndex>
	class FEVisualisationEngine_CUDA : public FEVisualisationEngine < TVoxel, TIndex >
	{
	private:
		uint *noTotalPoints_device;

	public:
		explicit FEVisualisationEngine_CUDA(FEScene<TVoxel, TIndex> *scene);
		~FEVisualisationEngine_CUDA(void);

		void FindVisibleBlocks(const FEPose *pose, const FEIntrinsics *intrinsics, FERenderState *renderState) const;
		void CreateExpectedDepths(const FEPose *pose, const FEIntrinsics *intrinsics, FERenderState *renderState) const;
		void RenderImage(const FEPose *pose, const FEIntrinsics *intrinsics, const FERenderState *renderState,
			UChar4Image *outputImage, PFEVisualisationEngine::RenderImageType type = PFEVisualisationEngine::RENDER_SHADED_GREYSCALE) const;
		void FindSurface(const FEPose *pose, const FEIntrinsics *intrinsics, const FERenderState *renderState) const;
		void CreateICPMaps(const FEView *view, FETrackingState *trackingState, FERenderState *renderState) const;
		void ForwardRender(const FEView *view, FETrackingState *trackingState, FERenderState *renderState) const;
		void RenderCurrentView(const FEView *view, const Matrix4f &M_d, FERenderState *renderState) const;

		FERenderState* CreateRenderState(const Vector2i & imgSize) const;
	};

	template<class TVoxel>
	class FEVisualisationEngine_CUDA<TVoxel, FEVoxelBlockHash> : public FEVisualisationEngine < TVoxel, FEVoxelBlockHash >
	{
	private:
		uint *noTotalPoints_device;
		RenderingBlock *renderingBlockList_device;
		uint *noTotalBlocks_device;
		int *noVisibleEntries_device;
	public:
		explicit FEVisualisationEngine_CUDA(FEScene<TVoxel, FEVoxelBlockHash> *scene);
		~FEVisualisationEngine_CUDA(void);

		void FindVisibleBlocks(const FEPose *pose, const FEIntrinsics *intrinsics, FERenderState *renderState) const;
		void CreateExpectedDepths(const FEPose *pose, const FEIntrinsics *intrinsics, FERenderState *renderState) const;
		void RenderImage(const FEPose *pose, const FEIntrinsics *intrinsics, const FERenderState *renderState,
			UChar4Image *outputImage, PFEVisualisationEngine::RenderImageType type = PFEVisualisationEngine::RENDER_SHADED_GREYSCALE) const;
		void FindSurface(const FEPose *pose, const FEIntrinsics *intrinsics, const FERenderState *renderState) const;
		void CreateICPMaps(const FEView *view, FETrackingState *trackingState, FERenderState *renderState) const;
		void ForwardRender(const FEView *view, FETrackingState *trackingState, FERenderState *renderState) const;
		void RenderCurrentView(const FEView *view, const Matrix4f &M_d, FERenderState *renderState) const;

		FERenderState_VH* CreateRenderState(const Vector2i & imgSize) const;
	};
}
#endif // _FE_VISUALISATIONENGINE_CUDA_H
