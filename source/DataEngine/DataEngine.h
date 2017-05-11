// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#ifndef _DATAENGINE_H
#define _DATAENGINE_H

#include <vector>
#include <memory>
#include <string>
#include <opencv2/core/core.hpp>

#include "Define.h"

//parent class to acquire rgbd images
class DataEngine
{
public:
	typedef std::shared_ptr<DataEngine> Ptr;
	typedef std::shared_ptr<DataEngine const> ConstPtr;

    DataEngine();
    virtual ~DataEngine() {}

    virtual bool hasMoreImages(void) = 0;
	virtual bool getNewImages(void) = 0;
	UChar4Image* getCurrentRgbImage();
	ShortImage* getCurrentDepthImage();

	cv::Mat getCurrentMatRgbImage();
	cv::Mat getCurrentMatDepthImage();

    Vector2i getDepthImageSize();
    Vector2i getRGBImageSize();

	UChar4ImagesBlock *getRgbImagesBlock();
	ShortImagesBlock *getDepthImagesBlock();

	int getCurrentFrameId();

	void readCameraPoses(std::string filename);
	const std::vector<Matrix4f>& getAllCameraPoses() const;
	const std::vector<int>& getAllFlags() const;
	const Matrix4f getCameraPoses(int i) const;

protected:
	UChar4Image *rgbImage;
	ShortImage *rawDepthImage;
	UChar4ImagesBlock *rgbImagesBlock;
	ShortImagesBlock *depthImagesBlock;
	
	cv::Mat matRgbImage;
	cv::Mat matDepthImage;

	int image_width;
	int image_height;
	int curFrameId;

	std::vector<Matrix4f> allCameraPoses;
	std::vector<int> allFlags;
};

#endif


