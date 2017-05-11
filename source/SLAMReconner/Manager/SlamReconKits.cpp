// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#include "SlamReconKits.h"
#include <QFileDialog>

SlamReconKits::SlamReconKits()
{
	dataEnginePtr = NULL;
	fusionCompoPtr = NULL;
	slamCompoPtr = NULL;
}

SlamReconKits::~SlamReconKits()
{

}

bool SlamReconKits::initKits(int device, int method)
{
	this->kInfo.reset();

	kInfo.deviceID = device;
	kInfo.methodID = method;

	QString slamSettingFile = QFileDialog::getOpenFileName(0,
		tr("load camera param file"), QString("../../data")
		);

	if (slamSettingFile.isEmpty())
		return false;

	bool flag = readCameraParam(slamSettingFile.toStdString());
	if (!flag){
		return false;
	}

	switch (device){
	case KitsInfo::KINECT1:
	{
		DataEngine *dataEngine = new OpenNIEngine();
		dataEnginePtr = DataEngine::Ptr(dataEngine);
		break;
	}
	case KitsInfo::FILES:
	{
		QString filesDirPath = QFileDialog::getExistingDirectory(0,
			tr("choose files' dir"), QString("../../data")
			);

		string assoFilePath = filesDirPath.toStdString() + "/associations.txt";

		DataEngine *dataEngine = new FileReaderEngine(filesDirPath.toStdString(), filesDirPath.toStdString(), assoFilePath, kInfo.imageSize[0], kInfo.imageSize[1]);
		dataEnginePtr = DataEngine::Ptr(dataEngine);
		break;
	}
	}

	if (method == KitsInfo::Method::SLAMRECON){
		Map *m_pMap = new Map();
		CovisibilityGraph *m_pCoGraph = new CovisibilityGraph();
		SpanningTree* m_pSpanTree = new SpanningTree(m_pCoGraph);
		SLAM *slamEngine = new SLAM("../../data/ORBvoc.txt", slamSettingFile.toStdString(), m_pMap, m_pCoGraph, m_pSpanTree);
		SLAMComponent *slamComponent = new SLAMComponent(m_pMap, m_pSpanTree, m_pCoGraph, slamEngine);
		slamCompoPtr = SLAMComponent::Ptr(slamComponent);
	}
	
	FELibSettings *internalSettings = new FELibSettings();
	FERGBDCalib *calib = new FERGBDCalib();
	calib->intrinsics_d = kInfo.intrinsics;
	FusionEngine *fusionEngine = new FusionEngine(internalSettings, calib, dataEnginePtr->getRGBImageSize(), dataEnginePtr->getDepthImageSize());
	FusionComponent *fusionComponent = new FusionComponent(fusionEngine, internalSettings, calib);
	fusionCompoPtr = FusionComponent::Ptr(fusionComponent);

	return true;
}

bool SlamReconKits::readCameraParam(const string &paramFile)
{
	cv::FileStorage fSettings(paramFile, cv::FileStorage::READ);
	if (!fSettings.isOpened())
	{
		cerr << "Failed to open settings file at: " << paramFile << endl;
		return false;
	}

	/*==================Camera calibration and image size===================*/
	kInfo.imageSize[0] = fSettings["Camera.width"];
	kInfo.imageSize[1] = fSettings["Camera.height"];
	kInfo.intrinsics.SetFrom(fSettings["Camera.fx"], fSettings["Camera.fy"], fSettings["Camera.cx"], fSettings["Camera.cy"]);

	fSettings.release();

	return true;
}
