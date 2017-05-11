// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM
// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#ifndef _FE_DENSEMAPPER_H
#define _FE_DENSEMAPPER_H

#include "../Utils/FELibDefines.h"
#include "../Utils/FELibSettings.h"

#include "../Objects/FEScene.h"
#include "../Objects/FETrackingState.h"
#include "../Objects/FERenderState.h"

#include "FESceneReconstructionEngine.h"
#include "FEVisualisationEngine.h"

namespace FE
{
	/** \brief
	*/
	template<class TVoxel, class TIndex>
	class FEDenseMapper
	{
	private:
		FESceneReconstructionEngine<TVoxel, TIndex> *sceneRecoEngine;

	public:
		void ResetScene(FEScene<TVoxel, TIndex> *scene);

		/// Process a single frame
		void ProcessFrame(const FEView *view, const FETrackingState *trackingState, FEScene<TVoxel, TIndex> *scene, FERenderState *renderState_live);

		/// Process a single frame given a specific camera pose
		void ProcessFrame(const FEView *view, const Matrix4f &M_d, FEScene<TVoxel, TIndex> *scene, FERenderState *renderState_live);

		/// Process a single frame given a specific camera pose and frame index
		void ProcessFrame(const FEView *view, const int index, const Matrix4f &M_d, FEScene<TVoxel, TIndex> *scene, FERenderState *renderState_live);

		/// reintegrate into scene
		void Reintegration(const FEView *view, const int frameIndex, const Matrix4f &old_M, const Matrix4f &new_M, FEScene<TVoxel, TIndex> *scene, FERenderState *renderState_live);

		/// Update the visible list (this can be called to update the visible list when fusion is turned off)
		void UpdateVisibleList(const FEView *view, const FETrackingState *trackingState, FEScene<TVoxel, TIndex> *scene, FERenderState *renderState);

		/** \brief Constructor
			Ommitting a separate image size for the depth images
			will assume same resolution as for the RGB images.
			*/
		explicit FEDenseMapper(const FELibSettings *settings);
		~FEDenseMapper();
	};
}
#endif //_FE_DENSEMAPPER_H
