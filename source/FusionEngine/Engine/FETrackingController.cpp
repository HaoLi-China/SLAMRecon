// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM
#include "FETrackingController.h"
//#include "../Objects/FERenderState_VH.h"

using namespace FE;

void FETrackingController::Track(FETrackingState *trackingState, const FEView *view)
{
	if (trackingState->age_pointCloud != -1) tracker->TrackCamera(trackingState, view);

	trackingState->requiresFullRendering = trackingState->TrackerFarFromPointCloud() || !settings->useApproximateRaycast;
}

void FETrackingController::Prepare(FETrackingState *trackingState, const FEView *view, FERenderState *renderState)
{
	//render for tracking
	visualisationEngine->CreateExpectedDepths(trackingState->pose_d, &(view->calib->intrinsics_d), renderState);

	if (trackingState->requiresFullRendering)
	{
		visualisationEngine->CreateICPMaps(view, trackingState, renderState);
		trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);
		if (trackingState->age_pointCloud == -1) trackingState->age_pointCloud = -2;
		else trackingState->age_pointCloud = 0;
	}
	else
	{
		visualisationEngine->ForwardRender(view, trackingState, renderState);
		trackingState->age_pointCloud++;
	}
}
