//Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM.

#ifndef _PLATFORMINDEPENDENCE_H
#define _PLATFORMINDEPENDENCE_H

#include <cstdio>
#include <stdexcept>

#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
#define _CPU_AND_GPU_CODE_ __device__	// for CUDA device code
#else
#define _CPU_AND_GPU_CODE_ 
#endif

#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
#define _CPU_AND_GPU_CONSTANT_ __constant__	// for CUDA device code
#else
#define _CPU_AND_GPU_CONSTANT_
#endif


#define THREADPTR(x) x
#define DEVICEPTR(x) x
#define THREADGROUPPTR(x) x
#define CONSTPTR(x) x

#define DIEWITHEXCEPTION(x) throw std::runtime_error(x)

#endif