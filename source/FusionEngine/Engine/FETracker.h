// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM
#ifndef _FE_Tracker_H
#define _FE_Tracker_H

#include "../Utils/FELibDefines.h"

#include "../Objects/FETrackingState.h"
#include "../Objects/FEView.h"

namespace FE
{
	/** \brief
		Basic interface to any sort of trackers that will align an
		incoming view with an existing scene.
		*/
	class FETracker
	{
	public:
		/** Localize a View in the given scene. The result is
			currently stored as an attribute in trackingState.
			*/
		virtual void TrackCamera(FETrackingState *trackingState, const FEView *view) = 0;

		/** Updates the initial pose of the depth camera in the scene.
			This can be used to make the scene up vector correspond
			to the real world's up direction.
			*/
		virtual void UpdateInitialPose(FETrackingState *trackingState) {}

		virtual ~FETracker(void) {}
	};
}

#endif //_FE_Tracker_H