// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM
#include "FELibSettings.h"

#include <stdio.h>

using namespace FE;

FELibSettings::FELibSettings(void)
	: sceneParams(0.02f, 32766, 0.005f, 0.2f, 3.0f, false)
{
	/// depth threashold for the ICP tracker
	depthTrackerICPThreshold = 0.1f * 0.1f;

	/// For ITMDepthTracker: ICP iteration termination threshold
	depthTrackerTerminationThreshold = 1e-3f;

	/// skips every other point when using the colour tracker
	skipPoints = true;

	deviceType = DEVICE_CUDA;

	/// enables or disables approximate raycast
	useApproximateRaycast = false;

	/// enable or disable bilateral depth filtering;
	useBilateralFilter = false;

	trackerType = TRACKER_ICP;

	/// model the sensor noise as  the weight for weighted ICP
	modelSensorNoise = false;

	{
		noHierarchyLevels = 5;
		trackingRegime = new TrackerIterationType[noHierarchyLevels];

		trackingRegime[0] = TRACKER_ITERATION_BOTH;
		trackingRegime[1] = TRACKER_ITERATION_BOTH;
		trackingRegime[2] = TRACKER_ITERATION_ROTATION;
		trackingRegime[3] = TRACKER_ITERATION_ROTATION;
		trackingRegime[4] = TRACKER_ITERATION_ROTATION;
	}

	noICPRunTillLevel = 0;
}

FELibSettings::~FELibSettings()
{
	delete[] trackingRegime;
}
