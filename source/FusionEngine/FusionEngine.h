// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM
// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#ifndef _FE_FUSIONENGINE_H
#define _FE_FUSIONENGINE_H

#include "Utils/FELibSettings.h"
#include "Objects/FEScene.h"
#include "Objects/FEMesh.h"
#include "Objects/FEView.h"
#include "Objects/FERGBDCalib.h"
#include "Engine/FELowLevelEngine.h"
#include "Engine/FEVisualisationEngine.h"
#include "Engine/FETrackingController.h"
#include "Engine/FEMeshingEngine.h"
#include "Engine/FEViewBuilder.h"
#include "Engine/FEDenseMapper.h"

#include <vector>

namespace FE
{
	class FusionEngine
	{
	public:
		/** \brief Constructor
		Ommitting a separate image size for the depth images
		will assume same resolution as for the RGB images.
		*/
		FusionEngine(const FELibSettings *settings, const FERGBDCalib *calib, const Vector2i imgSize_rgb = Vector2i(-1, -1), const Vector2i imgSize_d = Vector2i(-1, -1));
		~FusionEngine();

	private:
		const FELibSettings *settings;

		bool fusionActive, mainProcessingActive;

		FELowLevelEngine *lowLevelEngine;
		PFEVisualisationEngine *visualisationEngine;

		FEMeshingEngine<FEVoxel, FEVoxelIndex> *meshingEngine;
		FEMesh *mesh;

		FEViewBuilder *viewBuilder;
		FEDenseMapper<FEVoxel, FEVoxelIndex> *denseMapper;
		FETrackingController *trackingController;

		FETracker *tracker;

		FEView *view;
		FETrackingState *trackingState;

		FEScene<FEVoxel, FEVoxelIndex> *scene;
		FERenderState *renderState_live;
		FERenderState *renderState_freeview;

		FEPose freePose;
		Matrix4f currentM;

	public:
		enum GetImageType
		{
			IMAGE_ORIGINAL_RGB,
			IMAGE_ORIGINAL_DEPTH,
			IMAGE_SCENERAYCAST,
			IMAGE_FREECAMERA_CAST,
			IMAGE_UNKNOWN
		};

		/// Gives access to the current input frame
		FEView* GetView() { return view; }

		/// Gives access to the current camera pose and additional tracking information
		FETrackingState* GetTrackingState(void) { return trackingState; }

		/// Gives access to the internal world representation
		FEScene<FEVoxel, FEVoxelIndex>* GetScene(void) { return scene; }

		/// Process a frame with rgb and depth images and optionally a corresponding imu measurement
		void ProcessFrame(UChar4Image *rgbImage, ShortImage *rawDepthImage);

		/// Process a frame with rgb and depth images given a specific camera pose
		void ProcessFrame(UChar4Image *rgbImage, ShortImage *rawDepthImage, const Matrix4f &M_d);

		/// Process a frame with rgb and depth images given a specific camera pose and frameIndex
		void ProcessFrame(UChar4Image *rgbImage, ShortImage *rawDepthImage, const int index, const Matrix4f &M_d);

		/// ReIntegration
		void ReprocessFrame(UChar4Image *rgbImage, ShortImage *rawDepthImage, const int frameIndex, const Matrix4f &old_M, const Matrix4f &new_M);

		// Gives access to the data structure used internally to store any created meshes
		FEMesh* GetMesh(void) { return mesh; }

		/// Update the internally stored mesh data structure and return a pointer to it
		FEMesh* UpdateMesh(void);

		/// Extracts a mesh from the current scene and saves it to the obj file specified by the file name
		void SaveSceneToMesh(const char *objFileName);

		/// Get a result image as output
		Vector2i GetImageSize(void) const;

		void GetImage(UChar4Image *out, GetImageType getImageType, FEPose *pose = NULL, FEIntrinsics *intrinsics = NULL);

		/// switch for turning intergration on/off
		void turnOnIntegration();
		void turnOffIntegration();

		/// switch for turning main processing on/off
		void turnOnMainProcessing();
		void turnOffMainProcessing();

		//save surface points
		void SaveSurfacePoints(); 
		void getSurfacePoints(std::vector<Vector3f> &points, std::vector<Vector3f> &normals, std::vector<short> &sdf_s, const bool withNormals, const bool withSDFs);
	
		//get free pose
		FEPose getFreePose();

	    //set free pose
		void setFreePose(FEPose &freePose);

		//render free view
		void renderFreeView();

		//resetScene
		void resetScene();
	};
}

#endif //_FE_FUSIONENGINE_H 
