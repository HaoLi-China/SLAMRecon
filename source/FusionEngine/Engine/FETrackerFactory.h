// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM
#ifndef _FE_TRACKERFACTORY_H
#define _FE_TRACKERFACTORY_H

#include <map>
#include <stdexcept>

#include "FELowLevelEngine.h"
#include "FETracker.h"
#include "../Objects/FEScene.h"
#include "../Utils/FELibSettings.h"
#include "CUDA/FEDepthTracker_CUDA.h"

namespace FE
{
	/**
	 * \brief An instance of this class can be used to construct trackers.
	 */
	template <typename TVoxel, typename TIndex>
	class FETrackerFactory
	{
		//#################### TYPEDEFS ####################
	private:
		typedef FETracker *(*Maker)(const Vector2i&, const FELibSettings*, const FELowLevelEngine*, FEScene<TVoxel, TIndex>*);

		//#################### PRIVATE VARIABLES ####################
	private:
		/** A map of maker functions for the various different tracker types. */
		std::map<FELibSettings::TrackerType, Maker> makers;

		//#################### SINGLETON IMPLEMENTATION ####################
	private:
		/**
		 * \brief Constructs a tracker factory.
		 */
		FETrackerFactory()
		{
			makers.insert(std::make_pair(FELibSettings::TRACKER_ICP, &MakeICPTracker));
		}

	public:
		/**
		 * \brief Gets the singleton instance for the current set of template parameters.
		 */
		static FETrackerFactory& Instance()
		{
			static FETrackerFactory s_instance;
			return s_instance;
		}

		//#################### PUBLIC MEMBER FUNCTIONS ####################
	public:
		/**
		 * \brief Makes a tracker of the type specified in the settings.
		 */
		FETracker *Make(const Vector2i& trackedImageSize, const FELibSettings *settings, const FELowLevelEngine *lowLevelEngine,
			FEScene<TVoxel, TIndex> *scene) const
		{
			typename std::map<FELibSettings::TrackerType, Maker>::const_iterator it = makers.find(settings->trackerType);
			if (it == makers.end()) DIEWITHEXCEPTION("Unknown tracker type");

			Maker maker = it->second;
			return (*maker)(trackedImageSize, settings, lowLevelEngine, scene);
		}

		//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
	public:
		/**
		 * \brief Makes an ICP tracker.
		 */
		static FETracker *MakeICPTracker(const Vector2i& trackedImageSize, const FELibSettings *settings,
			const FELowLevelEngine *lowLevelEngine, FEScene<TVoxel, TIndex> *scene)
		{
			switch (settings->deviceType)
			{
			case FELibSettings::DEVICE_CUDA:
			{
				return new FEDepthTracker_CUDA(
					trackedImageSize,
					settings->trackingRegime,
					settings->noHierarchyLevels,
					settings->noICPRunTillLevel,
					settings->depthTrackerICPThreshold,
					settings->depthTrackerTerminationThreshold,
					lowLevelEngine
					);
			}
			default: break;
			}

			DIEWITHEXCEPTION("Failed to make ICP tracker");
		}

	};
}

#endif //_FE_TRACKERFACTORY_H