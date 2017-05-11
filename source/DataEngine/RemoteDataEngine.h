// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#ifndef _REMOTEDATAENGINE_H
#define _REMOTEDATAENGINE_H

#include <string>
#include "DSocket.h"
#include "DataEngine.h"

//engine to acquire rgbd images from remote server.
class RemoteDataEngine : public DataEngine
{
public:
	RemoteDataEngine(DSocket *dsocket);
	~RemoteDataEngine();

    bool hasMoreImages();
	bool getNewImages();

private:
	bool openCamera();
	bool closeCamera();

private:
	DSocket *dsocket;
};

#endif