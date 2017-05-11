/**
* Definitions about CUDA.
*
* Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM.
*/

#ifndef _CUDADEFINES_H
#define _CUDADEFINES_H

#include <cuda.h>
#include <cuda_runtime_api.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <stdio.h>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#  include <windows.h>
#endif

#ifndef BcudaSafeCall
#define BcudaSafeCall(err) Basis::__cudaSafeCall(err, __FILE__, __LINE__)

namespace Basis {

inline void __cudaSafeCall( cudaError err, const char *file, const int line )
{
    if( cudaSuccess != err) {
		printf("%s(%i) : cudaSafeCall() Runtime API error : %s.\n",
                file, line, cudaGetErrorString(err) );
        exit(-1);
    }
}
}

#endif
#endif//_CUDADEFINES_H

