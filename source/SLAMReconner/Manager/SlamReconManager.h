// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#ifndef _SLAMRECONMANAGER_H
#define _SLAMRECONMANAGER_H

#include <QObject>

#include "SlamReconKits.h"


using namespace FE;
using namespace SLAMRecon;

// class to manage all operations of the system.
class SlamReconManager : public QObject
{
	Q_OBJECT
public:
	SlamReconManager();
	~SlamReconManager();

public:
	void chooseKinectOne();
	void chooseFiles();

	void chooseSLAMRecon();
	void chooseKinectFusion();

	bool initSystem();

	bool startSystem();
	void stopSystem();
	void resetSystem();

	void saveMesh();

	int getDeviceFlag();
	int getMethodFlag();

private:
	void kinectFusionProcess();
	bool doKinectFusion();

	void fusionProcess();
	void slamReconProcess();
	bool doSlamRecon();

	void saveMeshForSLAMRecon();
	void saveMeshForKinfu();

signals:
	void updateFusionView();
	void updateSLAMView();

public:
	SlamReconKits::Ptr srkPtr;

private:
	int deviceFlag;
	int methodFlag;

	bool m_bInit;
	bool m_ShutdowmFlag;
	bool m_FusionFlag;

	int stopFlag;
	int resetFlag;

	bool allDone;
};

#endif // _SLAMRECONMANAGER_H

