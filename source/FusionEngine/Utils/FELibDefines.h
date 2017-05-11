// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM
#ifndef _FE_LIBDEFINE_H
#define _FE_LIBDEFINE_H

#ifdef NDEBUG
#undef NDEBUG
#include <assert.h>
#define NDEBUG
#else
#include <assert.h>
#endif // NDEBUG

#ifndef FESafeCall
#define FESafeCall BcudaSafeCall
#endif

#include <cuda_runtime.h>

#include "Define.h"
//////////////////////////////////////////////////////////////////////////
// Voxel Hashing definition and helper functions
//////////////////////////////////////////////////////////////////////////

#define SDF_BLOCK_SIZE 8				// SDF block size
#define SDF_BLOCK_SIZE3 512				// SDF_BLOCK_SIZE3 = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE
#define SDF_LOCAL_BLOCK_NUM 0x40000		// Number of locally stored blocks, currently 2^17

#define SDF_GLOBAL_BLOCK_NUM 0x120000	// Number of globally stored blocks: SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE
#define SDF_TRANSFER_BLOCK_NUM 0x1000	// Maximum number of blocks transfered in one swap operation

#define SDF_BUCKET_NUM 0x100000			// Number of Hash Bucket, should be 2^n and bigger than SDF_LOCAL_BLOCK_NUM, SDF_HASH_MASK = SDF_BUCKET_NUM - 1
#define SDF_HASH_MASK 0xfffff			// Used for get hashing value of the bucket index,  SDF_HASH_MASK = SDF_BUCKET_NUM - 1
#define SDF_EXCESS_LIST_SIZE 0x20000	// 0x20000 Size of excess list, used to handle collisions. Also max offset (unsigned short) value.

//////////////////////////////////////////////////////////////////////////
// Voxel Hashing data structures
//////////////////////////////////////////////////////////////////////////


//A single entry in the hash table.
struct FEHashEntry
{
	/** Position of the corner of the 8x8x8 volume, that identifies the entry. */
	Vector3s pos;
	/** Offset in the excess list. */
	int offset;
	/** Pointer to the voxel block array.
	    - >= 0 identifies an actual allocated entry in the voxel block array
	    - -1 identifies an entry that has been removed (swapped out)
	    - <-1 identifies an unallocated block
	*/
	int ptr;
};

#include "../Objects/FEVoxelBlockHash.h"
//#include "../Objects/FEPlainVoxelArray.h"

//Stores the information of a single voxel in the volume
struct FEVoxel_f_rgb
{
	_CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 1.0f; }
	_CPU_AND_GPU_CODE_ static float SDF_valueToFloat(float x) { return x; }
	_CPU_AND_GPU_CODE_ static float SDF_floatToValue(float x) { return x; }

	static const CONSTPTR(bool) hasColorInformation = true;

	/** Value of the truncated signed distance transformation. */
	float sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** RGB colour information stored for this voxel. */
	Vector3u clr;
	/** Number of observations that made up @p clr. */
	uchar w_color;

	_CPU_AND_GPU_CODE_ FEVoxel_f_rgb()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
		clr = (uchar)0;
		w_color = 0;
	}
};

struct FEVoxel_s_rgb
{
	_CPU_AND_GPU_CODE_ static short SDF_initialValue() { return 32767; }
	_CPU_AND_GPU_CODE_ static float SDF_valueToFloat(float x) { return (float)(x) / 32767.0f; }
	_CPU_AND_GPU_CODE_ static short SDF_floatToValue(float x) { return (short)((x) * 32767.0f); }

	static const CONSTPTR(bool) hasColorInformation = true;

	/** Value of the truncated signed distance transformation. */
	short sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** Padding that may or may not improve performance on certain GPUs */
	//uchar pad;
	/** RGB colour information stored for this voxel. */
	Vector3u clr;
	/** Number of observations that made up @p clr. */
	uchar w_color;

	_CPU_AND_GPU_CODE_ FEVoxel_s_rgb()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
		clr = (uchar)0;
		w_color = 0;
	}
};

struct FEVoxel_s
{
	_CPU_AND_GPU_CODE_ static short SDF_initialValue() { return 32767; }
	_CPU_AND_GPU_CODE_ static float SDF_valueToFloat(float x) { return (float)(x) / 32767.0f; }
	_CPU_AND_GPU_CODE_ static short SDF_floatToValue(float x) { return (short)((x) * 32767.0f); }

	static const CONSTPTR(bool) hasColorInformation = false;

	/** Value of the truncated signed distance transformation. */
	short sdf;
	/** Number of fused observations that make up @p sdf. */
	//uchar w_depth;
	short w_depth;
	/** Padding that may or may not improve performance on certain GPUs */
	//uchar pad;

	_CPU_AND_GPU_CODE_ FEVoxel_s()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
	}

	_CPU_AND_GPU_CODE_ void resetValue(){
		sdf = SDF_initialValue();
		w_depth = 0;
	}
};

struct FEVoxel_f
{
	_CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 1.0f; }
	_CPU_AND_GPU_CODE_ static float SDF_valueToFloat(float x) { return x; }
	_CPU_AND_GPU_CODE_ static float SDF_floatToValue(float x) { return x; }

	static const CONSTPTR(bool) hasColorInformation = false;

	/** Value of the truncated signed distance transformation. */
	float sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** Padding that may or may not improve performance on certain GPUs */
	//uchar pad;

	_CPU_AND_GPU_CODE_ FEVoxel_f()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
	}
};

/** This chooses the information stored at each voxel. At the moment, valid
    options are FEVoxel_s, FEVoxel_f, FEVoxel_s_rgb and FEVoxel_f_rgb 
*/
typedef FEVoxel_s FEVoxel;

/** This chooses the way the voxels are addressed and indexed. At the moment,
    valid options are FEVoxelBlockHash and FEPlainVoxelArray.
*/
typedef FE::FEVoxelBlockHash FEVoxelIndex;
//typedef ITMLib::Objects::ITMPlainVoxelArray ITMVoxelIndex;



#ifndef TRACKER_ITERATION_TYPE
#define TRACKER_ITERATION_TYPE
/// The tracker iteration type used to define the tracking iteration regime
typedef enum
{
	TRACKER_ITERATION_ROTATION = 1,
	TRACKER_ITERATION_TRANSLATION = 2,
	TRACKER_ITERATION_BOTH = 3,
	TRACKER_ITERATION_NONE = 4
} TrackerIterationType;
#endif

#endif//_FE_LIBDEFINE_H