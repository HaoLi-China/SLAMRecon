// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM.
// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#include "FusionEngine.h"

#include "Engine/CUDA/FEVisualisationEngine_CUDA.h"
#include "Engine/CUDA/FELowLevelEngine_CUDA.h"
#include "Engine/CUDA/FEMeshingEngine_CUDA.h"
#include "Engine/CUDA/FEViewBuilder_CUDA.h"
#include "PointsIO/PointsIO.h"
#include "Engine/Common/FECRepresentationAccess.h"

using namespace FE;

FusionEngine::FusionEngine(const FELibSettings *settings, const FERGBDCalib *calib, const Vector2i imgSize_rgb, const Vector2i imgSize_d){
	// create all the things required for marching cubes and mesh extraction
	// - uses additional memory (lots!)
	static const bool createMeshingEngine = true;

	this->settings = settings;
	this->scene = new FEScene<FEVoxel, FEVoxelIndex>(&(settings->sceneParams), MEMORYDEVICE_CUDA);

	meshingEngine = NULL;
	switch (settings->deviceType)
	{
	case FELibSettings::DEVICE_CUDA:
		lowLevelEngine = new FELowLevelEngine_CUDA();
		viewBuilder = new FEViewBuilder_CUDA(calib);
		visualisationEngine = new FEVisualisationEngine_CUDA<FEVoxel, FEVoxelIndex>(scene);
		if (createMeshingEngine) meshingEngine = new FEMeshingEngine_CUDA<FEVoxel, FEVoxelIndex>();
		break;
	}

	mesh = NULL;
	if (createMeshingEngine) mesh = new FEMesh(settings->deviceType == FELibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU);

	Vector2i trackedImageSize = FETrackingController::GetTrackedImageSize(settings, imgSize_rgb, imgSize_d);

	renderState_live = visualisationEngine->CreateRenderState(trackedImageSize);
	renderState_freeview = visualisationEngine->CreateRenderState(trackedImageSize);
	renderState_freeview->setRenderingRangeImage(trackedImageSize, 0.2f, 10.0f, MEMORYDEVICE_CUDA);

	denseMapper = new FEDenseMapper<FEVoxel, FEVoxelIndex>(settings);
	denseMapper->ResetScene(scene);

	tracker = FETrackerFactory<FEVoxel, FEVoxelIndex>::Instance().Make(trackedImageSize, settings, lowLevelEngine, scene);
	trackingController = new FETrackingController(tracker, visualisationEngine, lowLevelEngine, settings);

	trackingState = trackingController->BuildTrackingState(trackedImageSize);
	tracker->UpdateInitialPose(trackingState);

	view = NULL; // will be allocated by the view builder

	fusionActive = true;
	mainProcessingActive = true;

	Matrix4f idenM;
	idenM.setIdentity();
	freePose.SetM(idenM);

	currentM.setIdentity();
}

FusionEngine::~FusionEngine()
{
	if (renderState_live != NULL) delete renderState_live;
	if (renderState_freeview!=NULL) delete renderState_freeview;

	delete scene;

	delete denseMapper;
	delete trackingController;

	delete tracker;

	delete lowLevelEngine;
	delete viewBuilder;

	delete trackingState;
	if (view != NULL) delete view;

	delete visualisationEngine;

	if (meshingEngine != NULL) delete meshingEngine;

	if (mesh != NULL) delete mesh;
}

FEMesh* FusionEngine::UpdateMesh(void)
{
	if (mesh != NULL) meshingEngine->MeshScene(mesh, scene);
	return mesh;
}

void FusionEngine::SaveSceneToMesh(const char *objFileName)
{
	if (mesh == NULL) return;
	meshingEngine->MeshScene(mesh, scene);
	mesh->WriteSTL(objFileName);
}

void FusionEngine::ProcessFrame(UChar4Image *rgbImage, ShortImage *rawDepthImage)
{
	viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter,settings->modelSensorNoise);

	if (!mainProcessingActive) return;

	// tracking
	trackingController->Track(trackingState, view);

	// fusion
	if (fusionActive) denseMapper->ProcessFrame(view, trackingState, scene, renderState_live);

	// raycast to renderState_live for tracking and free visualisation
	currentM = trackingState->pose_d->GetM();
	trackingController->Prepare(trackingState, view, renderState_live);

	//raycast to renderState_freeview for visualisation
	visualisationEngine->RenderCurrentView(view, freePose.GetM(), renderState_freeview);
}

void FusionEngine::ProcessFrame(UChar4Image *rgbImage, ShortImage *rawDepthImage, const Matrix4f &M_d){
	viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, settings->modelSensorNoise);

	//fusion
	if (fusionActive) denseMapper->ProcessFrame(view, M_d, scene, renderState_live);

	//raycast to renderState_live for visualisation
	currentM = M_d;
	visualisationEngine->RenderCurrentView(view, M_d, renderState_live);

	//raycast to renderState_freeview for visualisation
	visualisationEngine->RenderCurrentView(view, freePose.GetM(), renderState_freeview);
}

void FusionEngine::ProcessFrame(UChar4Image *rgbImage, ShortImage *rawDepthImage, const int index, const Matrix4f &M_d){
	viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, settings->modelSensorNoise);

	//fusion
	if (fusionActive) denseMapper->ProcessFrame(view, index, M_d, scene, renderState_live);

	// raycast to renderState_live for visualisation
	currentM = M_d;
	visualisationEngine->RenderCurrentView(view, M_d, renderState_live);

	//raycast to renderState_freeview for visualisation
	visualisationEngine->RenderCurrentView(view, freePose.GetM(), renderState_freeview);
}

void FusionEngine::ReprocessFrame(UChar4Image *rgbImage, ShortImage *rawDepthImage, const int frameIndex, const Matrix4f &old_M, const Matrix4f &new_M){
	viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, settings->modelSensorNoise);

	//refusion
	if (fusionActive) denseMapper->Reintegration(view, frameIndex, old_M, new_M, scene, renderState_live);

	// raycast to renderState_live for visualisation
	visualisationEngine->RenderCurrentView(view, currentM, renderState_live);

	//raycast to renderState_freeview for visualisation
	visualisationEngine->RenderCurrentView(view, freePose.GetM(), renderState_freeview);
}

Vector2i FusionEngine::GetImageSize(void) const
{
	return renderState_live->raycastImage->noDims;
}

void FusionEngine::GetImage(UChar4Image *out, GetImageType getImageType, FEPose *pose, FEIntrinsics *intrinsics)
{
	if (view == NULL) return;

	out->Clear();

	switch (getImageType)
	{
	case FusionEngine::IMAGE_ORIGINAL_RGB:
		out->ChangeDims(view->rgb->noDims);
		if (settings->deviceType == FELibSettings::DEVICE_CUDA) 
			out->SetFrom(view->rgb, Basis::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(view->rgb, Basis::MemoryBlock<Vector4u>::CPU_TO_CPU);
		break;
	case FusionEngine::IMAGE_ORIGINAL_DEPTH:
		out->ChangeDims(view->depth->noDims);
		if (settings->deviceType == FELibSettings::DEVICE_CUDA) view->depth->UpdateHostFromDevice();
		FEVisualisationEngine<FEVoxel, FEVoxelIndex>::DepthToUchar4(out, view->depth);
		break;
	case FusionEngine::IMAGE_SCENERAYCAST:
	{
		Basis::Image<Vector4u> *srcImage = renderState_live->raycastImage;
		out->ChangeDims(srcImage->noDims);
		if (settings->deviceType == FELibSettings::DEVICE_CUDA)
			out->SetFrom(srcImage, Basis::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(srcImage, Basis::MemoryBlock<Vector4u>::CPU_TO_CPU);
		break;
	}
	case FusionEngine::IMAGE_FREECAMERA_CAST:
	{
		Basis::Image<Vector4u> *srcImage = renderState_freeview->raycastImage;
		out->ChangeDims(srcImage->noDims);
		if (settings->deviceType == FELibSettings::DEVICE_CUDA)
			out->SetFrom(srcImage, Basis::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(srcImage, Basis::MemoryBlock<Vector4u>::CPU_TO_CPU);
		break;
	}
	case FusionEngine::IMAGE_UNKNOWN:
		break;
	};
}

void FusionEngine::turnOnIntegration() { fusionActive = true; }
void FusionEngine::turnOffIntegration() { fusionActive = false; }
void FusionEngine::turnOnMainProcessing() { mainProcessingActive = true; }
void FusionEngine::turnOffMainProcessing() { mainProcessingActive = false; }

//get all surface points
void FusionEngine::getSurfacePoints(std::vector<Vector3f> &points, std::vector<Vector3f> &normals, std::vector<short> &sdf_s, const bool withNormals, const bool withSDFs){
	points.clear();
	normals.clear();

	Basis::MemoryBlock<FEHashEntry> *hashEntries = new Basis::MemoryBlock<FEHashEntry>(SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE, MEMORYDEVICE_CPU);
	FEHashEntry *hashTable = hashEntries->GetData(MEMORYDEVICE_CPU);
	FEVoxel *voxels = (FEVoxel*)malloc(SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(FEVoxel));

	FESafeCall(cudaMemcpy(hashTable, scene->index.GetEntries(), (SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE)*sizeof(FEHashEntry), cudaMemcpyDeviceToHost));
	FESafeCall(cudaMemcpy(voxels, scene->localVBA.GetVoxelBlocks(), SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3*sizeof(FEVoxel), cudaMemcpyDeviceToHost));

	float mu = scene->sceneParams->mu;

	for (int i = 0; i < SDF_BUCKET_NUM * 1 + SDF_EXCESS_LIST_SIZE; i++){
		const FEHashEntry &hashEntry = hashTable[i];

		if (hashEntry.ptr >= 0){
			for (int j = 0; j < SDF_BLOCK_SIZE3; j++){
				FEVoxel res = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + j];

				float value = FEVoxel::SDF_valueToFloat(res.sdf);
				if (value<10 * mu&&value>-10 * mu){ //mu=0.02
					if (withSDFs){
						sdf_s.push_back(res.sdf);
					}

					Vector3f p;
					float voxelSize = 0.125f;
					float blockSizeWorld = scene->sceneParams->voxelSize*SDF_BLOCK_SIZE; // = 0.005*8;

					p.z = (hashEntry.pos.z + (j / (SDF_BLOCK_SIZE*SDF_BLOCK_SIZE) + 0.5f)*voxelSize)*blockSizeWorld;
					p.y = (hashEntry.pos.y + ((j % (SDF_BLOCK_SIZE*SDF_BLOCK_SIZE)) / SDF_BLOCK_SIZE + 0.5f)*voxelSize)*blockSizeWorld;
					p.x = (hashEntry.pos.x + ((j % (SDF_BLOCK_SIZE*SDF_BLOCK_SIZE)) % SDF_BLOCK_SIZE + 0.5f)*voxelSize)*blockSizeWorld;

					if (withNormals){
						Vector3f n;

						Vector3f pt((hashEntry.pos.x*SDF_BLOCK_SIZE + ((j % (SDF_BLOCK_SIZE*SDF_BLOCK_SIZE)) % SDF_BLOCK_SIZE + 0.5f)), (hashEntry.pos.y*SDF_BLOCK_SIZE + ((j % (SDF_BLOCK_SIZE*SDF_BLOCK_SIZE)) / SDF_BLOCK_SIZE + 0.5f)), (hashEntry.pos.z*SDF_BLOCK_SIZE + (j / (SDF_BLOCK_SIZE*SDF_BLOCK_SIZE) + 0.5f)));

						Vector3f normal_host = computeSingleNormalFromSDF(voxels, hashTable, pt);

						float normScale = 1.0f / sqrtf(normal_host.x * normal_host.x + normal_host.y * normal_host.y + normal_host.z * normal_host.z);
						normal_host *= normScale;

						Vector3f pn(normal_host[0], normal_host[1], normal_host[2]);
						Vector3f tem(-p.x, -p.y, -p.z);

						double dotvalue = pn.x*tem.x + pn.y*tem.y + pn.z*tem.z;
						if (dotvalue < 0){
							pn = -pn;
						}

						n.x = pn.x;
						n.y = pn.y;
						n.z = pn.z;

						normals.push_back(n);
					}

					points.push_back(p);
				}
			}
		}
	}

	free(voxels);
	free(hashTable);
	voxels = NULL;
	hashTable = NULL;
}

void FusionEngine::SaveSurfacePoints(){
	std::vector<Vector3f> points;
	std::vector<Vector3f> normals;
	std::vector<short> sdf_s;
	getSurfacePoints(points, normals, sdf_s, true, false);

	PointsIO::savePLYfile("surface_points.ply", points, normals, Vector3u(255, 255, 255));
}

FEPose FusionEngine::getFreePose(){
	return this->freePose;
}

void FusionEngine::setFreePose(FEPose &freePose){
	this->freePose = freePose;
}

void FusionEngine::renderFreeView(){
	//raycast to renderState_freeview for visualisation
	visualisationEngine->RenderCurrentView(view, freePose.GetM(), renderState_freeview);
}

void FusionEngine::resetScene(){
	denseMapper->ResetScene(scene);
}