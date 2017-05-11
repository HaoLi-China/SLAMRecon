/**
* This file is a wrapper about an image block.
*
* Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
*/

#ifndef _IMAGESBLOCK_H
#define _IMAGESBLOCK_H

#include "MemoryBlock.h"
#include <iostream>

namespace Basis
{
	template <typename T>
	class ImagesBlock : public MemoryBlock < T >
	{
	public:
		/** Size of the image in pixels. */
		Vector2<int> noDims;

		/** Size of the ImagesBlock in images. */
		int size;

		/** Initialize an empty ImagesBlock of the given size, either
		on CPU only or on both CPU and GPU.
		*/
		ImagesBlock(Vector2<int> noDims, int size, bool allocate_CPU, bool allocate_CUDA, bool metalCompatible = true)
			: MemoryBlock<T>((size_t)(size) * noDims.x * noDims.y, allocate_CPU, allocate_CUDA, metalCompatible)
		{
			this->noDims = noDims;
			this->size = size;
		}

		ImagesBlock(bool allocate_CPU, bool allocate_CUDA, bool metalCompatible = true)
			: MemoryBlock<T>(0, allocate_CPU, allocate_CUDA, metalCompatible)
		{
			this->noDims = Vector2<int>(0, 0);
			this->size = 0;
		}

		ImagesBlock(Vector2<int> noDims, int size, MemoryDeviceType memoryType)
			: MemoryBlock<T>((size_t)(size) * noDims.x * noDims.y, memoryType)
		{
			this->noDims = noDims;
			this->size = size;
		}

		//save a image to cpu image block given an index
		bool saveImageToBlock(int index, Image<T> *img){
			if ((noDims.x != img->noDims.x) || (noDims.y != img->noDims.y)){
				return false;
			}

			if (index < size){
				DEVICEPTR(T)* imgCPU = img->GetData(MEMORYDEVICE_CPU);
				DEVICEPTR(T)* ibCPU = this->GetData(MEMORYDEVICE_CPU);
				memcpy(ibCPU + index*noDims.x*noDims.y, imgCPU, img->noDims.x*img->noDims.y*sizeof(T));

				return true;
			}
			return false;
		}

		//read a image to cpu from image block given an index
		bool readImageToCpu(int index, Image<T> *img){
			if ((noDims.x != img->noDims.x) || (noDims.y != img->noDims.y)){
				return false;
			}

			if (index < size){
				DEVICEPTR(T)* imgCPU = img->GetData(MEMORYDEVICE_CPU);
				DEVICEPTR(T)* ibCPU = this->GetData(MEMORYDEVICE_CPU);

				memcpy(imgCPU, ibCPU + index*noDims.x*noDims.y, img->noDims.x*img->noDims.y*sizeof(T));
				return true;
			}
			return false;
		}

		//read a image to gpu from image block given an index
		bool readImageToGpu(int index, Image<T> *img){
			if ((noDims.x != img->noDims.x) || (noDims.y != img->noDims.y)){
				return false;
			}

			if (index < size){
				DEVICEPTR(T)* imgGPU = img->GetData(MEMORYDEVICE_CUDA);
				DEVICEPTR(T)* ibCPU = this->GetData(MEMORYDEVICE_CPU);

				BcudaSafeCall(cudaMemcpy(imgGPU, ibCPU + index*noDims.x*noDims.y, img->noDims.x*img->noDims.y*sizeof(T), cudaMemcpyHostToDevice));
				return true;
			}
			return false;
		}
	};
}

#endif
