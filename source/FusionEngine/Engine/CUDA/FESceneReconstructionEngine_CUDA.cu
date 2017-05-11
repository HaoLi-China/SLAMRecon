// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM.
// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#include "FESceneReconstructionEngine_CUDA.h"
#include "FECUDAUtils.h"
#include "../Common/FECSceneReconstructionEngine.h"
#include "../../Objects/FERenderState_VH.h"

struct AllocationTempData {
	int noAllocatedVoxelEntries;
	int noAllocatedExcessEntries;
	int noVisibleEntries;
};

using namespace FE;

template<class TVoxel, bool stopMaxW, bool approximateIntegration>
__global__ void integrateIntoScene_device(TVoxel *localVBA, const FEHashEntry *hashTable, int *noVisibleEntryIDs,
	const Vector4u *rgb, Vector2i rgbImgSize, const float *depth, Vector2i imgSize, Matrix4f M_d, Matrix4f M_rgb, Vector4f projParams_d,
	Vector4f projParams_rgb, float _voxelSize, float mu, int maxW);

template<class TVoxel, bool stopMaxW, bool approximateIntegration>
__global__ void repealFromScene_device(TVoxel *localVBA, const FEHashEntry *hashTable, int *noVisibleEntryIDs,
	const Vector4u *rgb, Vector2i rgbImgSize, const float *depth, Vector2i imgSize, Matrix4f M_d, Matrix4f M_rgb, Vector4f projParams_d,
	Vector4f projParams_rgb, float _voxelSize, float mu, int maxW);

__global__ void buildHashAllocAndVisibleType_device(uchar *entriesAllocType, uchar *entriesVisibleType, Vector4s *blockCoords, const float *depth,
	Matrix4f invM_d, Vector4f projParams_d, float mu, Vector2i _imgSize, float _voxelSize, FEHashEntry *hashTable, float viewFrustum_min,
	float viewFrustrum_max);

__global__ void allocateVoxelBlocksList_device(int *voxelAllocationList, int *excessAllocationList, FEHashEntry *hashTable, int noTotalEntries,
	AllocationTempData *allocData, uchar *entriesAllocType, uchar *entriesVisibleType, Vector4s *blockCoords);

__global__ void reAllocateSwappedOutVoxelBlocks_device(int *voxelAllocationList, FEHashEntry *hashTable, int noTotalEntries,
	AllocationTempData *allocData, uchar *entriesVisibleType);

__global__ void setToType3(uchar *entriesVisibleType, int *visibleEntryIDs, int noVisibleEntries);

__global__ void buildVisibleList_device(FEHashEntry *hashTable, int noTotalEntries,
	int *visibleEntryIDs, AllocationTempData *allocData, uchar *entriesVisibleType, uchar *temStructure,
	Matrix4f M_d, Vector4f projParams_d, Vector2i depthImgSize, float voxelSize);

__global__ void buildBinaryVisibleList_device(int noTotalEntries, uchar *temStructure, uchar *visibleListPtr);

__global__ void buildVisibleListByBVLB_device(int noTotalEntries, int *visibleEntryIDs, uchar *entriesVisibleType, AllocationTempData *allocData, uchar *visibleListPtr);

// host methods

template<class TVoxel>
FESceneReconstructionEngine_CUDA<TVoxel, FEVoxelBlockHash>::FESceneReconstructionEngine_CUDA(void)
{
	FESafeCall(cudaMalloc((void**)&allocationTempData_device, sizeof(AllocationTempData)));
	FESafeCall(cudaMallocHost((void**)&allocationTempData_host, sizeof(AllocationTempData)));

	int noTotalEntries = FEVoxelBlockHash::noTotalEntries;
	FESafeCall(cudaMalloc((void**)&entriesAllocType_device, noTotalEntries));
	FESafeCall(cudaMalloc((void**)&blockCoords_device, noTotalEntries * sizeof(Vector4s)));

	FESafeCall(cudaMalloc((void**)&temStructure_device, (noTotalEntries+1) * sizeof(unsigned char)));
}

template<class TVoxel>
FESceneReconstructionEngine_CUDA<TVoxel, FEVoxelBlockHash>::~FESceneReconstructionEngine_CUDA(void)
{
	FESafeCall(cudaFreeHost(allocationTempData_host));
	FESafeCall(cudaFree(allocationTempData_device));
	FESafeCall(cudaFree(entriesAllocType_device));
	FESafeCall(cudaFree(blockCoords_device));
	FESafeCall(cudaFree(temStructure_device));
}

template<class TVoxel>
void FESceneReconstructionEngine_CUDA<TVoxel, FEVoxelBlockHash>::ResetScene(FEScene<TVoxel, FEVoxelBlockHash> *scene)
{
	int numBlocks = scene->index.getNumAllocatedVoxelBlocks();
	int blockSize = scene->index.getVoxelBlockSize();

	TVoxel *voxelBlocks_ptr = scene->localVBA.GetVoxelBlocks();
	memsetKernel<TVoxel>(voxelBlocks_ptr, TVoxel(), numBlocks * blockSize);
	int *vbaAllocationList_ptr = scene->localVBA.GetAllocationList();
	fillArrayKernel<int>(vbaAllocationList_ptr, numBlocks);
	scene->localVBA.lastFreeBlockId = numBlocks - 1;

	FEHashEntry tmpEntry;
	memset(&tmpEntry, 0, sizeof(FEHashEntry));
	tmpEntry.ptr = -2;
	FEHashEntry *hashEntry_ptr = scene->index.GetEntries();
	memsetKernel<FEHashEntry>(hashEntry_ptr, tmpEntry, scene->index.noTotalEntries);
	int *excessList_ptr = scene->index.GetExcessAllocationList();
	fillArrayKernel<int>(excessList_ptr, SDF_EXCESS_LIST_SIZE);

	scene->index.SetLastFreeExcessListId(SDF_EXCESS_LIST_SIZE - 1);
}

template<class TVoxel>
void FESceneReconstructionEngine_CUDA<TVoxel, FEVoxelBlockHash>::AllocateSceneFromDepth(FEScene<TVoxel, FEVoxelBlockHash> *scene, const FEView *view,
	const FETrackingState *trackingState, const FERenderState *renderState, bool onlyUpdateVisibleList)
{
	Matrix4f M_d = trackingState->pose_d->GetM();

	AllocateSceneFromDepth(scene, view, M_d, renderState, onlyUpdateVisibleList);
}

template<class TVoxel>
void FESceneReconstructionEngine_CUDA<TVoxel, FEVoxelBlockHash>::IntegrateIntoScene(FEScene<TVoxel, FEVoxelBlockHash> *scene, const FEView *view,
	const FETrackingState *trackingState, const FERenderState *renderState)
{
	Vector2i rgbImgSize = view->rgb->noDims;
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;

	Matrix4f M_d, M_rgb;
	Vector4f projParams_d, projParams_rgb;

	FERenderState_VH *renderState_vh = (FERenderState_VH*)renderState;

	M_d = trackingState->pose_d->GetM();
	if (TVoxel::hasColorInformation) M_rgb = view->calib->trafo_rgb_to_depth.calib_inv * M_d;

	projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
	projParams_rgb = view->calib->intrinsics_rgb.projectionParamsSimple.all;

	float mu = scene->sceneParams->mu; int maxW = scene->sceneParams->maxW;

	float *depth = view->depth->GetData(MEMORYDEVICE_CUDA);
	Vector4u *rgb = view->rgb->GetData(MEMORYDEVICE_CUDA);
	TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
	FEHashEntry *hashTable = scene->index.GetEntries();

	int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();

	dim3 cudaBlockSize(SDF_BLOCK_SIZE, SDF_BLOCK_SIZE, SDF_BLOCK_SIZE);
	dim3 gridSize(renderState_vh->noVisibleEntries);

	if (scene->sceneParams->stopIntegratingAtMaxW)
		if (trackingState->requiresFullRendering)
			integrateIntoScene_device<TVoxel, true, false> << <gridSize, cudaBlockSize >> >(localVBA, hashTable, visibleEntryIDs,
			rgb, rgbImgSize, depth, depthImgSize, M_d, M_rgb, projParams_d, projParams_rgb, voxelSize, mu, maxW);
		else
			integrateIntoScene_device<TVoxel, true, true> << <gridSize, cudaBlockSize >> >(localVBA, hashTable, visibleEntryIDs,
			rgb, rgbImgSize, depth, depthImgSize, M_d, M_rgb, projParams_d, projParams_rgb, voxelSize, mu, maxW);
	else
		if (trackingState->requiresFullRendering)
			integrateIntoScene_device<TVoxel, false, false> << <gridSize, cudaBlockSize >> >(localVBA, hashTable, visibleEntryIDs,
			rgb, rgbImgSize, depth, depthImgSize, M_d, M_rgb, projParams_d, projParams_rgb, voxelSize, mu, maxW);
		else
			integrateIntoScene_device<TVoxel, false, true> << <gridSize, cudaBlockSize >> >(localVBA, hashTable, visibleEntryIDs,
			rgb, rgbImgSize, depth, depthImgSize, M_d, M_rgb, projParams_d, projParams_rgb, voxelSize, mu, maxW);
}

template<class TVoxel>
void FESceneReconstructionEngine_CUDA<TVoxel, FEVoxelBlockHash>::AllocateSceneFromDepth(FEScene<TVoxel, FEVoxelBlockHash> *scene, const FEView *view, const Matrix4f &M_d,
	const FERenderState *renderState, bool onlyUpdateVisibleList = false)
{
	AllocateSceneFromDepth(scene, view, -1, M_d, renderState, onlyUpdateVisibleList);
}

template<class TVoxel>
void FESceneReconstructionEngine_CUDA<TVoxel, FEVoxelBlockHash>::AllocateSceneFromDepth(FEScene<TVoxel, FEVoxelBlockHash> *scene, const FEView *view, const int frameIndex, 
	const Matrix4f &M_d, const FERenderState *renderState, bool onlyUpdateVisibleList = false)
{
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;

	Matrix4f invM_d;
	Vector4f projParams_d, invProjParams_d;

	FERenderState_VH *renderState_vh = (FERenderState_VH*)renderState;
	M_d.inv(invM_d);

	projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
	invProjParams_d = projParams_d;
	invProjParams_d.x = 1.0f / invProjParams_d.x;
	invProjParams_d.y = 1.0f / invProjParams_d.y;

	float mu = scene->sceneParams->mu;

	float *depth = view->depth->GetData(MEMORYDEVICE_CUDA);
	int *voxelAllocationList = scene->localVBA.GetAllocationList();
	int *excessAllocationList = scene->index.GetExcessAllocationList();
	FEHashEntry *hashTable = scene->index.GetEntries();

	int noTotalEntries = scene->index.noTotalEntries;

	int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();
	uchar *entriesVisibleType = renderState_vh->GetEntriesVisibleType();
	uchar *visibleListPtr = renderState_vh->CetVisibleListPtr();

	dim3 cudaBlockSizeHV(16, 16);
	dim3 gridSizeHV((int)ceil((float)depthImgSize.x / (float)cudaBlockSizeHV.x), (int)ceil((float)depthImgSize.y / (float)cudaBlockSizeHV.y));

	dim3 cudaBlockSizeAL(256, 1);
	dim3 gridSizeAL((int)ceil((float)noTotalEntries / (float)cudaBlockSizeAL.x));

	dim3 cudaBlockSizeBVL(256, 1);
	dim3 gridSizeBVL((int)ceil((float)(noTotalEntries / 8 + 1) / (float)cudaBlockSizeAL.x));

	dim3 cudaBlockSizeVS(256, 1);
	dim3 gridSizeVS((int)ceil((float)renderState_vh->noVisibleEntries / (float)cudaBlockSizeVS.x));

	float oneOverVoxelSize = 1.0f / (voxelSize * SDF_BLOCK_SIZE);

	AllocationTempData *tempData = (AllocationTempData*)allocationTempData_host;
	tempData->noAllocatedVoxelEntries = scene->localVBA.lastFreeBlockId;
	tempData->noAllocatedExcessEntries = scene->index.GetLastFreeExcessListId();
	tempData->noVisibleEntries = 0;
	FESafeCall(cudaMemcpyAsync(allocationTempData_device, tempData, sizeof(AllocationTempData), cudaMemcpyHostToDevice));

	FESafeCall(cudaMemsetAsync(entriesAllocType_device, 0, sizeof(unsigned char)* noTotalEntries));

	FESafeCall(cudaMemsetAsync(visibleListPtr, 0, sizeof(uchar)* (noTotalEntries / 8 + 1)));

	FESafeCall(cudaMemsetAsync(entriesVisibleType, 0, sizeof(uchar)* noTotalEntries));

	if (gridSizeVS.x > 0) setToType3 << <gridSizeVS, cudaBlockSizeVS >> > (entriesVisibleType, visibleEntryIDs, renderState_vh->noVisibleEntries);

	buildHashAllocAndVisibleType_device << <gridSizeHV, cudaBlockSizeHV >> >(entriesAllocType_device, entriesVisibleType,
		blockCoords_device, depth, invM_d, invProjParams_d, mu, depthImgSize, oneOverVoxelSize, hashTable,
		scene->sceneParams->viewFrustum_min, scene->sceneParams->viewFrustum_max);

	if (!onlyUpdateVisibleList)
	{
		allocateVoxelBlocksList_device << <gridSizeAL, cudaBlockSizeAL >> >(voxelAllocationList, excessAllocationList, hashTable,
			noTotalEntries, (AllocationTempData*)allocationTempData_device, entriesAllocType_device, entriesVisibleType,
			blockCoords_device);
	}

	FESafeCall(cudaMemsetAsync(temStructure_device, 0, sizeof(unsigned char)* (noTotalEntries + 1)));
	buildVisibleList_device << <gridSizeAL, cudaBlockSizeAL >> >(hashTable, noTotalEntries, visibleEntryIDs,
		(AllocationTempData*)allocationTempData_device, entriesVisibleType, temStructure_device, M_d, projParams_d, depthImgSize, voxelSize);

	buildBinaryVisibleList_device << <gridSizeBVL, cudaBlockSizeBVL >> >(noTotalEntries, temStructure_device, visibleListPtr);

	FESafeCall(cudaMemcpy(tempData, allocationTempData_device, sizeof(AllocationTempData), cudaMemcpyDeviceToHost));
	renderState_vh->noVisibleEntries = tempData->noVisibleEntries;
	scene->localVBA.lastFreeBlockId = tempData->noAllocatedVoxelEntries;
	scene->index.SetLastFreeExcessListId(tempData->noAllocatedExcessEntries);

	if (frameIndex >= 0){
		Basis::MemoryBlock<uchar> *visibleList = renderState_vh->CetVisibleList();
		renderState_vh->GetVisibleListBlock()->saveVisibleListToBlock(frameIndex, visibleList, MEMORYDEVICE_CUDA);
	}
	else{
		Basis::MemoryBlock<uchar> *visibleList = renderState_vh->CetVisibleList();
		renderState_vh->GetVisibleListBlock()->saveVisibleListToBlock(visibleList, MEMORYDEVICE_CUDA);
	}
}

template<class TVoxel>
void FESceneReconstructionEngine_CUDA<TVoxel, FEVoxelBlockHash>::IntegrateIntoScene(FEScene<TVoxel, FEVoxelBlockHash> *scene, const FEView *view, const Matrix4f &M_d,
	const FERenderState *renderState)
{
	Vector2i rgbImgSize = view->rgb->noDims;
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;

	Matrix4f M_rgb;
	Vector4f projParams_d, projParams_rgb;

	FERenderState_VH *renderState_vh = (FERenderState_VH*)renderState;

	if (TVoxel::hasColorInformation) M_rgb = view->calib->trafo_rgb_to_depth.calib_inv * M_d;

	projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
	projParams_rgb = view->calib->intrinsics_rgb.projectionParamsSimple.all;

	float mu = scene->sceneParams->mu; int maxW = scene->sceneParams->maxW;

	float *depth = view->depth->GetData(MEMORYDEVICE_CUDA);
	Vector4u *rgb = view->rgb->GetData(MEMORYDEVICE_CUDA);
	TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
	FEHashEntry *hashTable = scene->index.GetEntries();

	int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();

	dim3 cudaBlockSize(SDF_BLOCK_SIZE, SDF_BLOCK_SIZE, SDF_BLOCK_SIZE);
	dim3 gridSize(renderState_vh->noVisibleEntries);

	if (scene->sceneParams->stopIntegratingAtMaxW)
		integrateIntoScene_device<TVoxel, true, false> << <gridSize, cudaBlockSize >> >(localVBA, hashTable, visibleEntryIDs,
		rgb, rgbImgSize, depth, depthImgSize, M_d, M_rgb, projParams_d, projParams_rgb, voxelSize, mu, maxW);
	else
		integrateIntoScene_device<TVoxel, false, false> << <gridSize, cudaBlockSize >> >(localVBA, hashTable, visibleEntryIDs,
		rgb, rgbImgSize, depth, depthImgSize, M_d, M_rgb, projParams_d, projParams_rgb, voxelSize, mu, maxW);
}

template<class TVoxel>
void FESceneReconstructionEngine_CUDA<TVoxel, FEVoxelBlockHash>::UpdateVisibleEntryIDsByBVLB(FEScene<TVoxel, FEVoxelBlockHash> *scene, const int index, const FERenderState *renderState){
	FERenderState_VH *renderState_vh = (FERenderState_VH*)renderState;

	int noTotalEntries = scene->index.noTotalEntries;

	int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();
	uchar *entriesVisibleType = renderState_vh->GetEntriesVisibleType();
	Basis::MemoryBlock<uchar> *visibleList = renderState_vh->CetVisibleList();
	renderState_vh->GetVisibleListBlock()->readVisibleListToGpu(index, visibleList);

	uchar *visibleListPtr = renderState_vh->CetVisibleListPtr();
	
	AllocationTempData *tempData = (AllocationTempData*)allocationTempData_host;
	tempData->noAllocatedVoxelEntries = scene->localVBA.lastFreeBlockId;
	tempData->noAllocatedExcessEntries = scene->index.GetLastFreeExcessListId();
	tempData->noVisibleEntries = 0;
	FESafeCall(cudaMemcpyAsync(allocationTempData_device, tempData, sizeof(AllocationTempData), cudaMemcpyHostToDevice));

	FESafeCall(cudaMemsetAsync(entriesVisibleType, 0, sizeof(uchar)* noTotalEntries));

	dim3 cudaBlockSize(256, 1);
	dim3 gridSize((int)ceil((float)noTotalEntries / (float)cudaBlockSize.x));
	buildVisibleListByBVLB_device << <gridSize, cudaBlockSize >> >(noTotalEntries, visibleEntryIDs, entriesVisibleType, (AllocationTempData*)allocationTempData_device, visibleListPtr);

	FESafeCall(cudaMemcpy(tempData, allocationTempData_device, sizeof(AllocationTempData), cudaMemcpyDeviceToHost));
	renderState_vh->noVisibleEntries = tempData->noVisibleEntries;
	scene->localVBA.lastFreeBlockId = tempData->noAllocatedVoxelEntries;
	scene->index.SetLastFreeExcessListId(tempData->noAllocatedExcessEntries);
}

template<class TVoxel>
void FESceneReconstructionEngine_CUDA<TVoxel, FEVoxelBlockHash>::RepealFromScene(FEScene<TVoxel, FEVoxelBlockHash> *scene, const FEView *view, const Matrix4f &M_d,
	const FERenderState *renderState){
	Vector2i rgbImgSize = view->rgb->noDims;
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;

	Matrix4f M_rgb;
	Vector4f projParams_d, projParams_rgb;

	FERenderState_VH *renderState_vh = (FERenderState_VH*)renderState;

	if (TVoxel::hasColorInformation) M_rgb = view->calib->trafo_rgb_to_depth.calib_inv * M_d;

	projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
	projParams_rgb = view->calib->intrinsics_rgb.projectionParamsSimple.all;

	float mu = scene->sceneParams->mu; int maxW = scene->sceneParams->maxW;

	float *depth = view->depth->GetData(MEMORYDEVICE_CUDA);
	Vector4u *rgb = view->rgb->GetData(MEMORYDEVICE_CUDA);
	TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
	FEHashEntry *hashTable = scene->index.GetEntries();

	int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();

	dim3 cudaBlockSize(SDF_BLOCK_SIZE, SDF_BLOCK_SIZE, SDF_BLOCK_SIZE);
	dim3 gridSize(renderState_vh->noVisibleEntries);

	repealFromScene_device<TVoxel, false, false> << <gridSize, cudaBlockSize >> >(localVBA, hashTable, visibleEntryIDs,
	rgb, rgbImgSize, depth, depthImgSize, M_d, M_rgb, projParams_d, projParams_rgb, voxelSize, mu, maxW);
}

//template<class TVoxel>
//void FESceneReconstructionEngine_CUDA<TVoxel, FEVoxelBlockHash>::testVLB(){
//
//}

template<class TVoxel, bool stopMaxW, bool approximateIntegration>
__global__ void integrateIntoScene_device(TVoxel *localVBA, const FEHashEntry *hashTable, int *visibleEntryIDs,
	const Vector4u *rgb, Vector2i rgbImgSize, const float *depth, Vector2i depthImgSize, Matrix4f M_d, Matrix4f M_rgb, Vector4f projParams_d,
	Vector4f projParams_rgb, float _voxelSize, float mu, int maxW)
{
	Vector3i globalPos;
	int entryId = visibleEntryIDs[blockIdx.x];

	const FEHashEntry &currentHashEntry = hashTable[entryId];

	if (currentHashEntry.ptr < 0) return;

	globalPos = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

	TVoxel *localVoxelBlock = &(localVBA[currentHashEntry.ptr * SDF_BLOCK_SIZE3]);

	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;

	Vector4f pt_model; int locId;

	locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

	if (stopMaxW) if (localVoxelBlock[locId].w_depth == maxW) return;
	if (approximateIntegration) if (localVoxelBlock[locId].w_depth != 0) return;

	pt_model.x = (float)(globalPos.x + x) * _voxelSize;
	pt_model.y = (float)(globalPos.y + y) * _voxelSize;
	pt_model.z = (float)(globalPos.z + z) * _voxelSize;
	pt_model.w = 1.0f;

	ComputeUpdatedVoxelInfo<TVoxel::hasColorInformation, TVoxel>::compute(localVoxelBlock[locId], pt_model, M_d, projParams_d, M_rgb, projParams_rgb, mu, maxW, depth, depthImgSize, rgb, rgbImgSize);
}

template<class TVoxel, bool stopMaxW, bool approximateIntegration>
__global__ void repealFromScene_device(TVoxel *localVBA, const FEHashEntry *hashTable, int *visibleEntryIDs,
	const Vector4u *rgb, Vector2i rgbImgSize, const float *depth, Vector2i depthImgSize, Matrix4f M_d, Matrix4f M_rgb, Vector4f projParams_d,
	Vector4f projParams_rgb, float _voxelSize, float mu, int maxW)
{
	Vector3i globalPos;
	int entryId = visibleEntryIDs[blockIdx.x];

	const FEHashEntry &currentHashEntry = hashTable[entryId];

	if (currentHashEntry.ptr < 0) return;

	globalPos = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

	TVoxel *localVoxelBlock = &(localVBA[currentHashEntry.ptr * SDF_BLOCK_SIZE3]);

	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;

	Vector4f pt_model; int locId;

	locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

	if (stopMaxW) if (localVoxelBlock[locId].w_depth == maxW) return;
	if (approximateIntegration) if (localVoxelBlock[locId].w_depth != 0) return;

	pt_model.x = (float)(globalPos.x + x) * _voxelSize;
	pt_model.y = (float)(globalPos.y + y) * _voxelSize;
	pt_model.z = (float)(globalPos.z + z) * _voxelSize;
	pt_model.w = 1.0f;

	recoverVoxelDepthInfo(localVoxelBlock[locId], pt_model, M_d, projParams_d, mu, maxW, depth, depthImgSize);
}

__global__ void buildHashAllocAndVisibleType_device(uchar *entriesAllocType, uchar *entriesVisibleType, Vector4s *blockCoords, const float *depth,
	Matrix4f invM_d, Vector4f projParams_d, float mu, Vector2i _imgSize, float _voxelSize, FEHashEntry *hashTable, float viewFrustum_min,
	float viewFrustum_max)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x > _imgSize.x - 1 || y > _imgSize.y - 1) return;

	buildHashAllocAndVisibleTypePP(entriesAllocType, entriesVisibleType, x, y, blockCoords, depth, invM_d,
		projParams_d, mu, _imgSize, _voxelSize, hashTable, viewFrustum_min, viewFrustum_max);
}

__global__ void setToType3(uchar *entriesVisibleType, int *visibleEntryIDs, int noVisibleEntries)
{
	int entryId = threadIdx.x + blockIdx.x * blockDim.x;
	if (entryId > noVisibleEntries - 1) return;
	entriesVisibleType[visibleEntryIDs[entryId]] = 3;
}

__global__ void allocateVoxelBlocksList_device(int *voxelAllocationList, int *excessAllocationList, FEHashEntry *hashTable, int noTotalEntries,
	AllocationTempData *allocData, uchar *entriesAllocType, uchar *entriesVisibleType, Vector4s *blockCoords)
{
	int targetIdx = threadIdx.x + blockIdx.x * blockDim.x;
	if (targetIdx > noTotalEntries - 1) return;

	int vbaIdx, exlIdx;

	switch (entriesAllocType[targetIdx])
	{
	case 1: //needs allocation, fits in the ordered list
		vbaIdx = atomicSub(&allocData->noAllocatedVoxelEntries, 1);

		if (vbaIdx >= 0) //there is room in the voxel block array
		{
			Vector4s pt_block_all = blockCoords[targetIdx];

			FEHashEntry hashEntry;
			hashEntry.pos.x = pt_block_all.x; hashEntry.pos.y = pt_block_all.y; hashEntry.pos.z = pt_block_all.z;
			hashEntry.ptr = voxelAllocationList[vbaIdx];
			hashEntry.offset = 0;

			hashTable[targetIdx] = hashEntry;
}
		break;

	case 2: //needs allocation in the excess list
		vbaIdx = atomicSub(&allocData->noAllocatedVoxelEntries, 1);
		exlIdx = atomicSub(&allocData->noAllocatedExcessEntries, 1);

		if (vbaIdx >= 0 && exlIdx >= 0) //there is room in the voxel block array and excess list
		{
			Vector4s pt_block_all = blockCoords[targetIdx];

			FEHashEntry hashEntry;
			hashEntry.pos.x = pt_block_all.x; hashEntry.pos.y = pt_block_all.y; hashEntry.pos.z = pt_block_all.z;
			hashEntry.ptr = voxelAllocationList[vbaIdx];
			hashEntry.offset = 0;

			int exlOffset = excessAllocationList[exlIdx];

			hashTable[targetIdx].offset = exlOffset + 1; //connect to child

			hashTable[SDF_BUCKET_NUM + exlOffset] = hashEntry; //add child to the excess list

			entriesVisibleType[SDF_BUCKET_NUM + exlOffset] = 1; //make child visible
		}

		break;
	}
}

__global__ void reAllocateSwappedOutVoxelBlocks_device(int *voxelAllocationList, FEHashEntry *hashTable, int noTotalEntries,
	AllocationTempData *allocData, /*int *noAllocatedVoxelEntries,*/ uchar *entriesVisibleType)
{
	int targetIdx = threadIdx.x + blockIdx.x * blockDim.x;
	if (targetIdx > noTotalEntries - 1) return;

	int vbaIdx;
	int hashEntry_ptr = hashTable[targetIdx].ptr;

	if (entriesVisibleType[targetIdx] > 0 && hashEntry_ptr == -1) //it is visible and has been previously allocated inside the hash, but deallocated from VBA
	{
		vbaIdx = atomicSub(&allocData->noAllocatedVoxelEntries, 1);
		if (vbaIdx >= 0) hashTable[targetIdx].ptr = voxelAllocationList[vbaIdx];
	}
}

__global__ void buildVisibleList_device(FEHashEntry *hashTable, int noTotalEntries,
	int *visibleEntryIDs, AllocationTempData *allocData, uchar *entriesVisibleType, uchar *temStructure,
	Matrix4f M_d, Vector4f projParams_d, Vector2i depthImgSize, float voxelSize)
{
	int targetIdx = threadIdx.x + blockIdx.x * blockDim.x;
	if (targetIdx > noTotalEntries - 1) return;

	__shared__ bool shouldPrefix;
	shouldPrefix = false;
	__syncthreads();

	unsigned char hashVisibleType = entriesVisibleType[targetIdx];
	const FEHashEntry & hashEntry = hashTable[targetIdx];

	if (hashVisibleType == 3)
	{
		//bool isVisibleEnlarged, isVisible;

		//checkBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d, voxelSize, depthImgSize);
		//if (!isVisible) hashVisibleType = 0;

		//entriesVisibleType[targetIdx] = hashVisibleType;

		hashVisibleType = 0;
		entriesVisibleType[targetIdx] = 0;
	}

	if (hashVisibleType > 0) {
		int index = targetIdx / 8;
		int offset = targetIdx % 8;

		unsigned char value = 1;
		for (int i = 1; i <= offset; i++){
			value *= 2;
		}

		temStructure[index * 8 + offset] = value;
		//visibleListPtr[index] += value;
		shouldPrefix = true;
	}

	__syncthreads();

	if (shouldPrefix)
	{
		int offset = computePrefixSum_device<int>(hashVisibleType > 0, &allocData->noVisibleEntries, blockDim.x * blockDim.y, threadIdx.x);
		if (offset != -1) visibleEntryIDs[offset] = targetIdx;
	}

#if 0
	// "active list": blocks that have new information from depth image
	// currently not used...
	__syncthreads();

	if (shouldPrefix)
	{
		int offset = computePrefixSum_device<int>(hashVisibleType == 1, noActiveEntries, blockDim.x * blockDim.y, threadIdx.x);
		if (offset != -1) activeEntryIDs[offset] = targetIdx;
	}
#endif
}

__global__ void buildBinaryVisibleList_device(int noTotalEntries, uchar *temStructure, uchar *visibleListPtr){
	int targetIdx = threadIdx.x + blockIdx.x * blockDim.x;
	if (targetIdx > noTotalEntries / 8) return;

	__syncthreads();
	unsigned char value = 0;

	for (int i = 0; i < 8; i++){
		value += temStructure[targetIdx * 8 + i];
	}

	visibleListPtr[targetIdx] = value;

	__syncthreads();
}

__global__ void buildVisibleListByBVLB_device(int noTotalEntries, int *visibleEntryIDs, uchar *entriesVisibleType, AllocationTempData *allocData, uchar *visibleListPtr){
	int targetIdx = threadIdx.x + blockIdx.x * blockDim.x;
	if (targetIdx > noTotalEntries - 1) return;

	__shared__ bool shouldPrefix;
	shouldPrefix = false;
	__syncthreads();

	int index = targetIdx / 8;
	unsigned char offset = targetIdx % 8;

	unsigned char vb = visibleListPtr[index];
	unsigned char tem = vb >> offset;

	unsigned char hashVisibleType = tem % 2;

	if (hashVisibleType > 0) {
		entriesVisibleType[targetIdx] = 6;
		shouldPrefix = true;
	}
	else{
		entriesVisibleType[targetIdx] = 0;
	}

	__syncthreads();

	if (shouldPrefix)
	{
		int offset = computePrefixSum_device<int>(hashVisibleType > 0, &allocData->noVisibleEntries, blockDim.x * blockDim.y, threadIdx.x);
		if (offset != -1) visibleEntryIDs[offset] = targetIdx;
	}
}

template class FE::FESceneReconstructionEngine_CUDA<FEVoxel, FEVoxelIndex>;
