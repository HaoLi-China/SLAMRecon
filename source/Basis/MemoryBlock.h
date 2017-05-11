/**
* This file defines the memory operation about GPU and CPU.
*
* Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM.
*/

#ifndef _MEMORYBLOCK_H
#define _MEMORYBLOCK_H

#include "CUDADefines.h"
#include "PlatformIndependence.h"

#include <stdlib.h>
#include <string.h>

#ifndef MEMORY_DEVICE_TYPE
#define MEMORY_DEVICE_TYPE
enum MemoryDeviceType { MEMORYDEVICE_CPU, MEMORYDEVICE_CUDA };
#endif 

namespace Basis
{
	/** \brief
	Represents memory blocks, templated on the data type
	*/
	template <typename T>
	class MemoryBlock
	{
	protected:
		bool isAllocated_CPU, isAllocated_CUDA, isMetalCompatible;

		/** Pointer to memory on CPU host. */
		DEVICEPTR(T)* data_cpu;

		/** Pointer to memory on GPU, if available. */
		DEVICEPTR(T)* data_cuda;

	public:
		enum MemoryCopyDirection { CPU_TO_CPU, CPU_TO_CUDA, CUDA_TO_CPU, CUDA_TO_CUDA };

		/** Total number of allocated entries in the data array. */
		size_t dataSize;

		/** Get the data pointer on CPU or GPU. */
		inline DEVICEPTR(T)* GetData(MemoryDeviceType memoryType)
		{
			switch (memoryType)
			{
			case MEMORYDEVICE_CPU: return data_cpu;
			case MEMORYDEVICE_CUDA: return data_cuda;
			}

			return 0;
		}

		/** Get the data pointer on CPU or GPU. */
		inline const DEVICEPTR(T)* GetData(MemoryDeviceType memoryType) const
		{
			switch (memoryType)
			{
			case MEMORYDEVICE_CPU: return data_cpu;
			case MEMORYDEVICE_CUDA: return data_cuda;
			}

			return 0;
		}

	/** Initialize an empty memory block of the given size,
		on CPU only or GPU only or on both. CPU might also use the
		Metal compatible allocator (i.e. with 16384 alignment).
		*/
		MemoryBlock(size_t dataSize, bool allocate_CPU, bool allocate_CUDA, bool metalCompatible = true)
		{
			this->isAllocated_CPU = false;
			this->isAllocated_CUDA = false;
			this->isMetalCompatible = false;

			Allocate(dataSize, allocate_CPU, allocate_CUDA, metalCompatible);
			Clear();
		}

		/** Initialize an empty memory block of the given size, either
		on CPU only or on GPU only. CPU will be Metal compatible if Metal
		is enabled.
		*/
		MemoryBlock(size_t dataSize, MemoryDeviceType memoryType)
		{
			this->isAllocated_CPU = false;
			this->isAllocated_CUDA = false;
			this->isMetalCompatible = false;

			switch (memoryType)
			{
			case MEMORYDEVICE_CPU: Allocate(dataSize, true, false, false); break;
			case MEMORYDEVICE_CUDA: Allocate(dataSize, false, true, false); break;
			}

			Clear();
		}

		/** Set all image data to the given @p defaultValue. */
		void Clear(unsigned char defaultValue = 0)
		{
			if (isAllocated_CPU) memset(data_cpu, defaultValue, dataSize * sizeof(T));
			if (isAllocated_CUDA) BcudaSafeCall(cudaMemset(data_cuda, defaultValue, dataSize * sizeof(T)));
		}

		/** Transfer data from CPU to GPU, if possible. */
		void UpdateDeviceFromHost() const {
			if (isAllocated_CUDA && isAllocated_CPU)
				BcudaSafeCall(cudaMemcpy(data_cuda, data_cpu, dataSize * sizeof(T), cudaMemcpyHostToDevice));
		}
		/** Transfer data from GPU to CPU, if possible. */
		void UpdateHostFromDevice() const {
			if (isAllocated_CUDA && isAllocated_CPU)
				BcudaSafeCall(cudaMemcpy(data_cpu, data_cuda, dataSize * sizeof(T), cudaMemcpyDeviceToHost));
		}

		/** Copy data */
		void SetFrom(const MemoryBlock<T> *source, MemoryCopyDirection memoryCopyDirection)
		{
			switch (memoryCopyDirection)
			{
			case CPU_TO_CPU:
				memcpy(this->data_cpu, source->data_cpu, source->dataSize * sizeof(T));
				break;
			case CPU_TO_CUDA:
				BcudaSafeCall(cudaMemcpyAsync(this->data_cuda, source->data_cpu, source->dataSize * sizeof(T), cudaMemcpyHostToDevice));
				break;
			case CUDA_TO_CPU:
				BcudaSafeCall(cudaMemcpy(this->data_cpu, source->data_cuda, source->dataSize * sizeof(T), cudaMemcpyDeviceToHost));
				break;
			case CUDA_TO_CUDA:
				BcudaSafeCall(cudaMemcpyAsync(this->data_cuda, source->data_cuda, source->dataSize * sizeof(T), cudaMemcpyDeviceToDevice));
				break;
			default: break;
			}
		}

		virtual ~MemoryBlock() { this->Free(); }

		/** Allocate image data of the specified size. If the
		data has been allocated before, the data is freed.
		*/
		void Allocate(size_t dataSize, bool allocate_CPU, bool allocate_CUDA, bool metalCompatible)
		{
			Free();

			this->dataSize = dataSize;

			if (allocate_CPU)
			{
				int allocType = 0;
				if (allocate_CUDA) allocType = 1;

				switch (allocType)
				{
				case 0:
					if (dataSize == 0) data_cpu = NULL;
					//else data_cpu = new T[dataSize];
					else {
						data_cpu = (DEVICEPTR(T)*) malloc(dataSize*sizeof(T));
					}
						
					break;
				case 1:
					if (dataSize == 0) data_cpu = NULL;
					else BcudaSafeCall(cudaMallocHost((void**)&data_cpu, dataSize * sizeof(T)));
					break;
				}

				this->isAllocated_CPU = allocate_CPU;
				this->isMetalCompatible = metalCompatible;
			}

			if (allocate_CUDA)
			{
				if (dataSize == 0) data_cuda = NULL;
				else BcudaSafeCall(cudaMalloc((void**)&data_cuda, dataSize * sizeof(T)));
				this->isAllocated_CUDA = allocate_CUDA;
			}
		}

		void Free()
		{
			if (isAllocated_CPU)
			{
				int allocType = 0;
				if (isAllocated_CUDA) allocType = 1;
				switch (allocType)
				{
				case 0:
					//if (data_cpu != NULL) delete[] data_cpu;
					if (data_cpu != NULL) free(data_cpu);
					break;
				case 1:
					if (data_cpu != NULL) BcudaSafeCall(cudaFreeHost(data_cpu));
					break;
				}

				isMetalCompatible = false;
				isAllocated_CPU = false;
			}

			if (isAllocated_CUDA)
			{
				if (data_cuda != NULL) BcudaSafeCall(cudaFree(data_cuda));
				isAllocated_CUDA = false;
			}
		}

		// Suppress the default copy constructor and assignment operator
		MemoryBlock(const MemoryBlock&);
		MemoryBlock& operator=(const MemoryBlock&);
	};
}

#endif
