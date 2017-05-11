// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#include <iostream>
#include "RemoteDataEngine.h"

using namespace std;

RemoteDataEngine::RemoteDataEngine(DSocket *dsocket){
	this->dsocket = dsocket;

	if (!dsocket->openSocket())
		printf("disconnected\n");

	openCamera();

	image_height = dsocket->doGetImageHeight();
	image_width = dsocket->doGetImageWidth();

	matRgbImage.create(image_width, image_height, CV_8UC3);
	matDepthImage.create(image_width, image_height, CV_16UC1);

	rgbImage = new UChar4Image(Vector2i(image_width, image_height), true, true);
	rawDepthImage = new ShortImage(Vector2i(image_width, image_height), true, true);

#ifdef USE_IMAGES_BLOCK
	//rgbImagesBlock = new UChar4ImagesBlock(Vector2i(image_width, image_height), IMAGES_BLOCK_SIZE, MEMORYDEVICE_CPU);
	depthImagesBlock = new ShortImagesBlock(Vector2i(image_width, image_height), IMAGES_BLOCK_SIZE, MEMORYDEVICE_CPU);
#else
	rgbImagesBlock = NULL;
	depthImagesBlock = NULL;
#endif // USE_IMAGES_BLOCK

	curFrameId = -1;
}

RemoteDataEngine::~RemoteDataEngine(){
	closeCamera();
}

bool RemoteDataEngine::hasMoreImages(){
	return true;
}

bool RemoteDataEngine::getNewImages(){
	Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
	short *depth = rawDepthImage->GetData(MEMORYDEVICE_CPU);

	bool success = false;
	while (!success){
		success = dsocket->doGetNewImages(rgb, depth, image_width, image_height);

		if (!success){
			continue;
		}

		for (int yc = 0; yc < image_height; yc++)
		{
			for (int xc = 0; xc < image_width; xc++)
			{
				int ind = yc*image_width + xc;
				matDepthImage.ptr<ushort>(yc)[xc] = depth[ind] * 5;
				matRgbImage.ptr<uchar>(yc)[xc * 3 + 2] = rgb[ind][0];
				matRgbImage.ptr<uchar>(yc)[xc * 3 + 1] = rgb[ind][1];
				matRgbImage.ptr<uchar>(yc)[xc * 3] = rgb[ind][2];
			}
		}
	}

	//bool success = false;
	//while (!success)
	//	success = dsocket->doGetNewImages(rgb, depth);
	//while (1)
	//	dsocket->doGetNewImages(rgb, depth);

	curFrameId++;

#ifdef USE_IMAGES_BLOCK
	//rgbImagesBlock->saveImageToBlock(curFrameId, rgbImage);
	depthImagesBlock->saveImageToBlock(curFrameId, rawDepthImage);
#endif

	return true;
}

bool RemoteDataEngine::openCamera(){
	return dsocket->doOpenCamera();
}

bool RemoteDataEngine::closeCamera(){
	return dsocket->doCloseCamera();
}

