// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM.
// Modified by authors of SLAMRecon.
#ifndef _FE_VISUALISATION_H
#define _FE_VISUALISATION_H

#include "../Utils/FELibDefines.h"

#include "../Objects/FEScene.h"
#include "../Objects/FEView.h"
#include "../Objects/FETrackingState.h"
#include "../Objects/FERenderState_VH.h"
#include "../Objects/FERenderState.h"
#include "../Objects/FEPose.h"

namespace FE
{
	class PFEVisualisationEngine
	{
	public:
		enum RenderImageType
		{
			RENDER_SHADED_GREYSCALE,
			RENDER_COLOUR_FROM_VOLUME,
			RENDER_COLOUR_FROM_NORMAL
		};

		virtual ~PFEVisualisationEngine(void) {}

		static void DepthToUchar4(UChar4Image *dst, FloatImage *src);
		static void NormalToUchar4(UChar4Image* dst, Float4Image *src);
		static void WeightToUchar4(UChar4Image *dst, FloatImage *src);

		/** Given a scene, pose and intrinsics, compute the
		visible subset of the scene and store it in an
		appropriate visualisation state object, created
		previously using allocateInternalState().
		*/
		virtual void FindVisibleBlocks(const FEPose *pose, const FEIntrinsics *intrinsics,
			FERenderState *renderState) const = 0;

		/** Given scene, pose and intrinsics, create an estimate
		of the minimum and maximum depths at each pixel of
		an image.
		*/
		virtual void CreateExpectedDepths(const FEPose *pose, const FEIntrinsics *intrinsics,
			FERenderState *renderState) const = 0;

		/** This will render an image using raycasting. */
		virtual void RenderImage(const FEPose *pose, const FEIntrinsics *intrinsics,
			const FERenderState *renderState, UChar4Image *outputImage, RenderImageType type = RENDER_SHADED_GREYSCALE) const = 0;

		/** Finds the scene surface using raycasting. */
		virtual void FindSurface(const FEPose *pose, const FEIntrinsics *intrinsics,
			const FERenderState *renderState) const = 0;

		/** Create an image of reference points and normals as
		required by the FE::FEDepthTracker classes.
		*/
		virtual void CreateICPMaps(const FEView *view, FETrackingState *trackingState,
			FERenderState *renderState) const = 0;

		/** Create an image of reference points and normals as
		required by the FE::FEDepthTracker classes.

		Incrementally previous raycast result.
		*/
		virtual void ForwardRender(const FEView *view, FETrackingState *trackingState,
			FERenderState *renderState) const = 0;

		//render current view
		virtual void RenderCurrentView(const FEView *view, const Matrix4f &M_d, FERenderState *renderState) const = 0;

		/** Creates a render state, containing rendering info
		for the scene.
		*/
		virtual FERenderState* CreateRenderState(const Vector2i & imgSize) const = 0;
	};

	template<class TIndex> struct IndexToRenderState { typedef FERenderState type; };
	template<> struct IndexToRenderState<FEVoxelBlockHash> { typedef FERenderState_VH type; };

	/** \brief
		Interface to engines helping with the visualisation of
		the results from the rest of the library.

		This is also used internally to get depth estimates for the
		raycasting done for the trackers. The basic idea there is
		to project down a scene of 8x8x8 voxel
		blocks and look at the bounding boxes. The projection
		provides an idea of the possible depth range for each pixel
		in an image, which can be used to speed up raycasting
		operations.
		*/
	template<class TVoxel, class TIndex>
	class FEVisualisationEngine : public PFEVisualisationEngine
	{
	protected:
		const FEScene<TVoxel, TIndex> *scene;
		FEVisualisationEngine(const FEScene<TVoxel, TIndex> *scene)
		{
			this->scene = scene;
		}
	public:
		/** Override */
		virtual typename IndexToRenderState<TIndex>::type *CreateRenderState(const Vector2i & imgSize) const = 0;
	};
}

#endif //_FE_VISUALISATION_H
