// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM
#ifndef _FE_SCENEHIERARCHYLEVEL_H
#define _FE_SCENEHIERARCHYLEVEL_H

#include "../Utils/FELibDefines.h"

namespace FE
{
	class FESceneHierarchyLevel
	{
	public:
		int levelId;

		TrackerIterationType iterationType;

		Float4Image *pointsMap;
		Float4Image *normalsMap;
		Vector4f intrinsics;

		bool manageData;

		FESceneHierarchyLevel(Vector2i imgSize, int levelId, TrackerIterationType iterationType, MemoryDeviceType memoryType, bool skipAllocation = false)
		{
			this->manageData = !skipAllocation;
			this->levelId = levelId;
			this->iterationType = iterationType;

			if (!skipAllocation) {
				this->pointsMap = new Float4Image(imgSize, memoryType);
				this->normalsMap = new Float4Image(imgSize, memoryType);
			}
		}

		void UpdateHostFromDevice()
		{
			this->pointsMap->UpdateHostFromDevice();
			this->normalsMap->UpdateHostFromDevice();
		}

		void UpdateDeviceFromHost()
		{
			this->pointsMap->UpdateDeviceFromHost();
			this->normalsMap->UpdateDeviceFromHost();
		}

		~FESceneHierarchyLevel(void)
		{
			if (manageData) {
				delete pointsMap;
				delete normalsMap;
			}
		}

		// Suppress the default copy constructor and assignment operator
		FESceneHierarchyLevel(const FESceneHierarchyLevel&);
		FESceneHierarchyLevel& operator=(const FESceneHierarchyLevel&);
	};
}

#endif //_FE_SCENEHIERARCHYLEVEL_H
