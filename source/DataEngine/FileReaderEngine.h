// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#ifndef _FILEREADERENGINE_H
#define _FILEREADERENGINE_H

#include <string>

#include "Calibration.h"
#include "DataEngine.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

//engine to acquire rgbd images from files
class FileReaderEngine : public DataEngine
{

public:
	FileReaderEngine(const std::string rgbPath, const std::string depthPath, const std::string assoFilePath, const int frame_width, const int frame_height);
	~FileReaderEngine();

	bool hasMoreImages();
	bool getNewImages();

private:
	int image_num;
	std::vector<string> rgbFileLists;
	std::vector<string> depthFileLists;

	std::string m_sRgbPath;
	std::string m_sDepthPath;

	std::vector<string> scanDirectory(const string path, const string extension);
	void getFiles(const string assoFilePath);
};

#endif