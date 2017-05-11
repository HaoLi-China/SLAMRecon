// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM
#ifndef _FE_TEMPLATEDHIERARCHYLEVEL_H
#define _FE_TEMPLATEDHIERARCHYLEVEL_H

namespace FE
{
	template <class ImageType>
	class FETemplatedHierarchyLevel
	{
	public:
		int levelId;

		TrackerIterationType iterationType;

		ImageType *depth;
		Vector4f intrinsics;
		bool manageData;

		FETemplatedHierarchyLevel(Vector2i imgSize, int levelId, TrackerIterationType iterationType,
			MemoryDeviceType memoryType, bool skipAllocation = false)
		{
			this->manageData = !skipAllocation;
			this->levelId = levelId;
			this->iterationType = iterationType;

			if (!skipAllocation) this->depth = new ImageType(imgSize, memoryType);
		}

		void UpdateHostFromDevice()
		{
			this->depth->UpdateHostFromDevice();
		}

		void UpdateDeviceFromHost()
		{
			this->depth->UpdateHostFromDevice();
		}

		~FETemplatedHierarchyLevel(void)
		{
			if (manageData) delete depth;
		}

		// Suppress the default copy constructor and assignment operator
		FETemplatedHierarchyLevel(const FETemplatedHierarchyLevel&);
		FETemplatedHierarchyLevel& operator=(const FETemplatedHierarchyLevel&);
	};
}
#endif //_FE_TEMPLATEDHIERARCHYLEVEL_H
