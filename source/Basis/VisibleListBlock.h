/**
* This file is a wrapper about an visible list block.
*
* Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
*/

#ifndef _VISIBLELISTBLOCK_H
#define _VISIBLELISTBLOCK_H

#include "MemoryBlock.h"
#include <iostream>

namespace Basis
{
	/** \brief
	Represents images, templated on the pixel type
	*/
	template <typename T>
	class VisibleListBlock : public MemoryBlock < T >
	{
	public:
		/** Length of the VisibleList. */
		size_t visibleListSize;

		/** Length of the VisibleListBlock. */
		int visibleListBlockSize;

		/** offset of the VisibleList. */
		int offset;

		/** Initialize an empty ImagesBlock of the given size, either
		on CPU only or on both CPU and GPU.
		*/
		VisibleListBlock(size_t visibleListSize, int visibleListBlockSize, bool allocate_CPU, bool allocate_CUDA, bool metalCompatible = true)
			: MemoryBlock<T>((size_t)(visibleListBlockSize) * visibleListSize, allocate_CPU, allocate_CUDA, metalCompatible)
		{
			this->visibleListSize = visibleListSize;
			this->visibleListBlockSize = visibleListBlockSize;
			offset = 0;
		}

		VisibleListBlock(bool allocate_CPU, bool allocate_CUDA, bool metalCompatible = true)
			: MemoryBlock<T>(0, allocate_CPU, allocate_CUDA, metalCompatible)
		{
			this->visibleListSize = 0;
			this->visibleListBlockSize = 0;
			offset = 0;
		}

		VisibleListBlock(size_t visibleListSize, int visibleListBlockSize, MemoryDeviceType memoryType)
			: MemoryBlock<T>((size_t)(visibleListBlockSize) * visibleListSize, memoryType)
		{
			this->visibleListSize = visibleListSize;
			this->visibleListBlockSize = visibleListBlockSize;
			offset = 0;
		}

		//save a visible list to cpu/gpu visible list block given an index
		bool saveVisibleListToBlock(int index, MemoryBlock <T> *visibleList, MemoryDeviceType memoryType){
			if (visibleListSize != visibleList->dataSize){
				return false;
			}

			if (index < visibleListBlockSize){
				DEVICEPTR(T)* vl = visibleList->GetData(memoryType);
				DEVICEPTR(T)* vlb= this->GetData(MEMORYDEVICE_CPU);

				switch (memoryType){
				case MEMORYDEVICE_CPU:
					memcpy(vlb + index*visibleListSize, vl, visibleList->dataSize*sizeof(T));
					break;

				case MEMORYDEVICE_CUDA:
					BcudaSafeCall(cudaMemcpy(vlb + index*visibleListSize, vl, visibleList->dataSize*sizeof(T), cudaMemcpyDeviceToHost));
					break;
				default:
					memcpy(vlb + index*visibleListSize, vl, visibleList->dataSize*sizeof(T));
					break;
				}

				offset++;

				return true;
			}
			return false;
		}

		bool saveVisibleListToBlock(MemoryBlock <T> *visibleList, MemoryDeviceType memoryType){
			return saveVisibleListToBlock(offset, visibleList, memoryType);
		}

		// read a visible list to cpu from visible list block given an index
		bool readVisibleListToCpu(int index, MemoryBlock <T> *visibleList){
			if (visibleListSize != visibleList->dataSize){
				return false;
			}

			if (index < visibleListBlockSize){
				DEVICEPTR(T)* vlCPU = visibleList->GetData(MEMORYDEVICE_CPU);
				DEVICEPTR(T)* vlbCPU = this->GetData(MEMORYDEVICE_CPU);

				memcpy(vlCPU, vlbCPU + index*visibleListSize, visibleList->dataSize*sizeof(T));
				return true;
			}
			return false;
		}

		// read a visible list to gpu from visible list block given an index
		bool readVisibleListToGpu(int index, MemoryBlock <T> *visibleList){
			if (visibleListSize != visibleList->dataSize){
				return false;
			}

			if (index < visibleListBlockSize){
				DEVICEPTR(T)* vlGPU = visibleList->GetData(MEMORYDEVICE_CUDA);
				DEVICEPTR(T)* vlbCPU = this->GetData(MEMORYDEVICE_CPU);

				BcudaSafeCall(cudaMemcpy(vlGPU, vlbCPU + index*visibleListSize, visibleList->dataSize*sizeof(T), cudaMemcpyHostToDevice));
				return true;
			}
			return false;
		}
	};
}

#endif
