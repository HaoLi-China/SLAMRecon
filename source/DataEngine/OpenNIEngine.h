// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#ifndef _OPENNIENGINE_H
#define _OPENNIENGINE_H

#include "OpenNI.h"
#include "DataEngine.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace openni;
using namespace cv;

//engine to acquire rgbd images from device supporting openni
class OpenNIEngine : public DataEngine
{

public:
    OpenNIEngine();
    ~OpenNIEngine();

    bool hasMoreImages();
	bool getNewImages();

private:
	bool openCamera();
	bool closeCamera();

private:
    VideoStream oniDepthStream;
    VideoStream oniColorStream;
    Device device;

    VideoFrameRef oniDepthImg;
    VideoFrameRef oniColorImg;
};

#endif

