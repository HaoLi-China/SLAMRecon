// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM.
#ifndef _FE_DEPTHTRACKER_H
#define _FE_DEPTHTRACKER_H

#include "../Utils/FELibDefines.h"
#include "Image.h"

#include "../Objects/FEImageHierarchy.h"
#include "../Objects/FETemplatedHierarchyLevel.h"
#include "../Objects/FESceneHierarchyLevel.h"

#include "FETracker.h"
#include "FELowLevelEngine.h"

namespace FE
{
	/** Base class for engine performing ICP based depth tracking.
		A typical example would be the original KinectFusion
		tracking algorithm.
		*/
	class FEDepthTracker : public FETracker
	{
	private:
		const FELowLevelEngine *lowLevelEngine;
		FEImageHierarchy<FESceneHierarchyLevel> *sceneHierarchy;
		FEImageHierarchy<FETemplatedHierarchyLevel<FloatImage> > *viewHierarchy;

		FETrackingState *trackingState; const FEView *view;

		int *noIterationsPerLevel;
		int noICPLevel;

		float terminationThreshold;

		void PrepareForEvaluation();
		void SetEvaluationParams(int levelId);

		void ComputeDelta(float *delta, float *nabla, float *hessian, bool shortIteration) const;
		void ApplyDelta(const Matrix4f & para_old, const float *delta, Matrix4f & para_new) const;
		bool HasConverged(float *step) const;

		void SetEvaluationData(FETrackingState *trackingState, const FEView *view);
	protected:
		float *distThresh;

		int levelId;
		TrackerIterationType iterationType;

		Matrix4f scenePose;
		FESceneHierarchyLevel *sceneHierarchyLevel;
		FETemplatedHierarchyLevel<FloatImage> *viewHierarchyLevel;

		virtual int ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose) = 0;

	public:
		void TrackCamera(FETrackingState *trackingState, const FEView *view);

		FEDepthTracker(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel, float distThresh,
			float terminationThreshold, const FELowLevelEngine *lowLevelEngine, MemoryDeviceType memoryType);
		virtual ~FEDepthTracker(void);
	};
}
#endif //_FE_DEPTHTRACKER_H
