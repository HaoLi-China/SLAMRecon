// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM
#ifndef _FE_DEPTHTRACKER_CUDA_H
#define _FE_DEPTHTRACKER_CUDA_H

#include "../FEDepthTracker.h"

namespace FE
{
	class FEDepthTracker_CUDA : public FEDepthTracker
	{
	public:
		struct AccuCell;

	private:
		AccuCell *accu_host;
		AccuCell *accu_device;

	protected:
		int ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose);

	public:
		FEDepthTracker_CUDA(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel, float distThresh,
			float terminationThreshold, const FELowLevelEngine *lowLevelEngine);
		~FEDepthTracker_CUDA(void);
	};
}
#endif //_FE_DEPTHTRACKER_CUDA_H
