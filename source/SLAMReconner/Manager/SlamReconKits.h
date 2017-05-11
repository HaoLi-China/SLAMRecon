/**
*This file defines all of kits for SLAMRecon
*Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
*/

#ifndef _SLAMRECONKITS_H
#define _SLAMRECONKITS_H

#include <memory>
#include <QObject>

#include "DSocket.h"
#include "RemoteDataEngine.h"
#include "FileReaderEngine.h"
#include "OpenNIEngine.h"
#include "FusionEngine.h"

#include "../SLAMEngine/SLAM/SLAM.h"
#include "../SLAMEngine/SLAM/Map.h"

using namespace FE;
using namespace SLAMRecon;

//components used by fusion
struct FusionComponent
{
	typedef shared_ptr<FusionComponent> Ptr;
	typedef shared_ptr<FusionComponent const> ConstPtr;

	//fusion
	FusionEngine *fusionEngine;
	FELibSettings *internalSettings;
	FERGBDCalib *calib;

	FusionComponent(FusionEngine *fusionEngine, FELibSettings *internalSettings, FERGBDCalib *calib)
	{
		this->fusionEngine = fusionEngine;
		this->internalSettings = internalSettings;
		this->calib = calib;
	}

	~FusionComponent()
	{
		releaseAll();
	};

	void releaseAll()
	{
		if (calib != NULL){
			delete calib;
			calib = NULL;
		}

		if (internalSettings != NULL){
			delete internalSettings;
			internalSettings = NULL;
		}

		if (fusionEngine != NULL){
			delete fusionEngine;
			fusionEngine = NULL;
		}
	};
};

//components used by SLAM
struct SLAMComponent
{
	typedef shared_ptr<SLAMComponent> Ptr;
	typedef shared_ptr<SLAMComponent const> ConstPtr;

	//slam
	Map* m_pMap;
	SpanningTree* m_pSpanTree;
	CovisibilityGraph* m_pCoGraph;
	SLAM* slamEngine;

	SLAMComponent(Map* m_pMap, SpanningTree* m_pSpanTree, CovisibilityGraph* m_pCoGraph, SLAM* slamEngine)
	{
		this->m_pMap = m_pMap;
		this->m_pSpanTree = m_pSpanTree;
		this->m_pCoGraph = m_pCoGraph;
		this->slamEngine = slamEngine;
	};

	~SLAMComponent()
	{
		releaseAll();
	};

	void releaseAll()
	{
		if (m_pMap != NULL){
			delete m_pMap;
			m_pMap = NULL;
		}

		if (m_pSpanTree != NULL){
			delete m_pSpanTree;
			m_pSpanTree = NULL;
		}

		if (m_pCoGraph != NULL){
			delete m_pCoGraph;
			m_pCoGraph = NULL;
		}

		if (slamEngine != NULL){
			delete slamEngine;
			slamEngine = NULL;
		}
	}
};

//basic parameters for kits
struct KitsInfo
{
	enum Device
	{
		KINECT1, FILES
	};

	enum Method
	{
		SLAMRECON, KINFU
	};

	KitsInfo(){ deviceID = -1; methodID = -1; };

	int deviceID;
	int methodID;
	Vector2i imageSize;
	FEIntrinsics intrinsics;

	void reset()
	{
		deviceID = -1; 
		methodID = -1;
		imageSize = Vector2i(0, 0);
		intrinsics.SetFrom(580, 580, 320, 240);
	};
};

//all kits for SLAMRecon
class SlamReconKits : public QObject
{
public:
	typedef shared_ptr<SlamReconKits> Ptr;
	typedef shared_ptr<SlamReconKits const> ConstPtr;

	SlamReconKits();
	~SlamReconKits();

public:
	bool initKits(int device, int method); // init all kits

public:
	DataEngine::Ptr dataEnginePtr; 
	FusionComponent::Ptr fusionCompoPtr;
	SLAMComponent::Ptr slamCompoPtr;

	//DSocket::Ptr dsocketPtr;

	//vector<DataEngine::Ptr> dataEnginePtrs; // May have several data engines.
	//vector<DSocket::Ptr> dsocketPtrs; // May have several sockets. Just for communicating with robot. Useless if there's no robot participating. 
	//FusionComponent::Ptr fusionCompoPtr;
	//vector<SLAMComponent::Ptr> slamCompoPtrs; // May have several SLAMComponents. 

private:
	KitsInfo kInfo;

private:
	bool readCameraParam(const string &paramFile); //read camera parameters
};

#endif // _SLAMRECONKITS_H

