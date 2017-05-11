// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM
#include "FEMeshingEngine_CUDA.h"
#include "../Common/FECMeshingEngine.h"
#include "FECUDAUtils.h"

#include "CUDADefines.h"

using namespace FE;

template<class TVoxel>
__global__ void meshScene_device(FEMesh::Triangle *triangles, unsigned int *noTriangles_device, float factor, int noTotalEntries,
	int noMaxTriangles, const Vector4s *visibleBlockGlobalPos, const TVoxel *localVBA, const FEHashEntry *hashTable);

__global__ void findAllocateBlocks(Vector4s *visibleBlockGlobalPos, const FEHashEntry *hashTable, int noTotalEntries);

template<class TVoxel>
FEMeshingEngine_CUDA<TVoxel,FEVoxelBlockHash>::FEMeshingEngine_CUDA(void) 
{
	FESafeCall(cudaMalloc((void**)&visibleBlockGlobalPos_device, SDF_LOCAL_BLOCK_NUM * sizeof(Vector4s)));
	FESafeCall(cudaMalloc((void**)&noTriangles_device, sizeof(unsigned int)));
}

template<class TVoxel>
FEMeshingEngine_CUDA<TVoxel,FEVoxelBlockHash>::~FEMeshingEngine_CUDA(void) 
{
	FESafeCall(cudaFree(visibleBlockGlobalPos_device));
	FESafeCall(cudaFree(noTriangles_device));
}

template<class TVoxel>
void FEMeshingEngine_CUDA<TVoxel, FEVoxelBlockHash>::MeshScene(FEMesh *mesh, const FEScene<TVoxel, FEVoxelBlockHash> *scene)
{
	FEMesh::Triangle *triangles = mesh->triangles->GetData(MEMORYDEVICE_CUDA);
	const TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
	const FEHashEntry *hashTable = scene->index.GetEntries();

	int noMaxTriangles = mesh->noMaxTriangles, noTotalEntries = scene->index.noTotalEntries;
	float factor = scene->sceneParams->voxelSize;

	FESafeCall(cudaMemset(noTriangles_device, 0, sizeof(unsigned int)));
	FESafeCall(cudaMemset(visibleBlockGlobalPos_device, 0, sizeof(Vector4s) * SDF_LOCAL_BLOCK_NUM));

	{ // identify used voxel blocks
		dim3 cudaBlockSize(256); 
		dim3 gridSize((int)ceil((float)noTotalEntries / (float)cudaBlockSize.x));

		findAllocateBlocks << <gridSize, cudaBlockSize >> >(visibleBlockGlobalPos_device, hashTable, noTotalEntries);
	}

	{ // mesh used voxel blocks
		dim3 cudaBlockSize(SDF_BLOCK_SIZE, SDF_BLOCK_SIZE, SDF_BLOCK_SIZE);
		dim3 gridSize(SDF_LOCAL_BLOCK_NUM / 16, 16);

		meshScene_device<TVoxel> << <gridSize, cudaBlockSize >> >(triangles, noTriangles_device, factor, noTotalEntries, noMaxTriangles,
			visibleBlockGlobalPos_device, localVBA, hashTable);

		FESafeCall(cudaMemcpy(&mesh->noTotalTriangles, noTriangles_device, sizeof(unsigned int), cudaMemcpyDeviceToHost));
	}
}

__global__ void findAllocateBlocks(Vector4s *visibleBlockGlobalPos, const FEHashEntry *hashTable, int noTotalEntries)
{
	int entryId = threadIdx.x + blockIdx.x * blockDim.x;
	if (entryId > noTotalEntries - 1) return;

	const FEHashEntry &currentHashEntry = hashTable[entryId];

	if (currentHashEntry.ptr >= 0) 
		visibleBlockGlobalPos[currentHashEntry.ptr] = Vector4s(currentHashEntry.pos.x, currentHashEntry.pos.y, currentHashEntry.pos.z, 1);
}

template<class TVoxel>
__global__ void meshScene_device(FEMesh::Triangle *triangles, unsigned int *noTriangles_device, float factor, int noTotalEntries, 
	int noMaxTriangles, const Vector4s *visibleBlockGlobalPos, const TVoxel *localVBA, const FEHashEntry *hashTable)
{
	const Vector4s globalPos_4s = visibleBlockGlobalPos[blockIdx.x + gridDim.x * blockIdx.y];

	if (globalPos_4s.w == 0) return;

	Vector3i globalPos = Vector3i(globalPos_4s.x, globalPos_4s.y, globalPos_4s.z) * SDF_BLOCK_SIZE;

	Vector3f vertList[12];
	int cubeIndex = buildVertList(vertList, globalPos, Vector3i(threadIdx.x, threadIdx.y, threadIdx.z), localVBA, hashTable);

	if (cubeIndex < 0) return;

	for (int i = 0; triangleTable[cubeIndex][i] != -1; i += 3)
	{
		int triangleId = atomicAdd(noTriangles_device, 1);

		if (triangleId < noMaxTriangles - 1)
		{
			triangles[triangleId].p0 = vertList[triangleTable[cubeIndex][i]] * factor;
			triangles[triangleId].p1 = vertList[triangleTable[cubeIndex][i + 1]] * factor;
			triangles[triangleId].p2 = vertList[triangleTable[cubeIndex][i + 2]] * factor;
		}
	}
}

template class FE::FEMeshingEngine_CUDA<FEVoxel, FEVoxelIndex>;
