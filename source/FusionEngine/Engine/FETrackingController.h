// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM
#ifndef _FE_TRACKINGCONTROLLER_H
#define _FE_TRACKINGCONTROLLER_H

#include "../Utils/FELibDefines.h"

#include "../Objects/FETrackingState.h"
#include "../Objects/FERenderState.h"

#include "../Engine/FEVisualisationEngine.h"
#include "../Engine/FELowLevelEngine.h"

#include "FETrackerFactory.h"

namespace FE
{
	/** \brief
	*/
	class FETrackingController
	{
	private:
		const FELibSettings *settings;
		const PFEVisualisationEngine *visualisationEngine;
		const FELowLevelEngine *lowLevelEngine;

		FETracker *tracker;

		MemoryDeviceType memoryType;

	public:
		void Track(FETrackingState *trackingState, const FEView *view);
		void Prepare(FETrackingState *trackingState, const FEView *view, FERenderState *renderState);

		FETrackingController(FETracker *tracker, const PFEVisualisationEngine *visualisationEngine, const FELowLevelEngine *lowLevelEngine,
			const FELibSettings *settings)
		{
			this->tracker = tracker;
			this->settings = settings;
			this->visualisationEngine = visualisationEngine;
			this->lowLevelEngine = lowLevelEngine;

			memoryType = settings->deviceType == FELibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
		}

		FETrackingState *BuildTrackingState(const Vector2i & trackedImageSize) const
		{
			return new FETrackingState(trackedImageSize, memoryType);
		}

		static Vector2i GetTrackedImageSize(const FELibSettings *settings, const Vector2i& imgSize_rgb, const Vector2i& imgSize_d)
		{
			return settings->trackerType == FELibSettings::TRACKER_COLOR ? imgSize_rgb : imgSize_d;
		}

		// Suppress the default copy constructor and assignment operator
		FETrackingController(const FETrackingController&);
		FETrackingController& operator=(const FETrackingController&);
	};
}

#endif //_FE_TRACKINGCONTROLLER_H
