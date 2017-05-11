// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM.
#ifndef _FE_POINTCLOUD_H
#define _FE_POINTCLOUD_H

#include <stdlib.h>

#include "../Utils/FELibDefines.h"
#include "Image.h"

namespace FE
{
	class FEPointCloud
	{
	public:
		uint noTotalPoints;

		Basis::Image<Vector4f> *locations, *colours;

		explicit FEPointCloud(Vector2i imgSize, MemoryDeviceType memoryType)
		{
			this->noTotalPoints = 0;

			locations = new Basis::Image<Vector4f>(imgSize, memoryType);
			colours = new Basis::Image<Vector4f>(imgSize, memoryType);
		}

		void UpdateHostFromDevice()
		{
			this->locations->UpdateHostFromDevice();
			this->colours->UpdateHostFromDevice();
		}

		void UpdateDeviceFromHost()
		{
			this->locations->UpdateDeviceFromHost();
			this->colours->UpdateDeviceFromHost();
		}

		~FEPointCloud()
		{
			delete locations;
			delete colours;
		}

		// Suppress the default copy constructor and assignment operator
		FEPointCloud(const FEPointCloud&);
		FEPointCloud& operator=(const FEPointCloud&);
	};
}
#endif //_FE_POINTCLOUD_H
