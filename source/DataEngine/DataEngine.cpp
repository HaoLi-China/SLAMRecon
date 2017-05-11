// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#include "DataEngine.h"
#include <stdio.h>
#include <fstream>

DataEngine::DataEngine(){
	rgbImage = NULL;
	rawDepthImage = NULL;
	rgbImagesBlock = NULL;
	depthImagesBlock = NULL;
	curFrameId = 0;
}

UChar4Image* DataEngine::getCurrentRgbImage(){
	return rgbImage;
}

ShortImage* DataEngine::getCurrentDepthImage(){
	return rawDepthImage;
}

UChar4ImagesBlock* DataEngine::getRgbImagesBlock(){
	return rgbImagesBlock;
}

ShortImagesBlock* DataEngine::getDepthImagesBlock(){
	return depthImagesBlock;
}

int DataEngine::getCurrentFrameId(){
	return curFrameId;
}

Vector2i DataEngine::getDepthImageSize(void){
	return Vector2i(image_width, image_height);
}

Vector2i DataEngine::getRGBImageSize(void){
	return Vector2i(image_width, image_height);
}


cv::Mat DataEngine::getCurrentMatRgbImage() {
	return matRgbImage;
}

cv::Mat DataEngine::getCurrentMatDepthImage(){
	return matDepthImage;
}

void DataEngine::readCameraPoses(std::string filename)
{
	std::ifstream in(filename);

	if (!in.is_open()){
		std::cout << "Fail to open file " << filename << std::endl;
		return;
	}

	int frameNum = 0;
	in >> frameNum;

	for (int i = 0; i < frameNum; i++){
		Matrix4f mat;
		mat.setIdentity();

		int flag;
		in >> mat(0, 0) >> mat(1, 0) >> mat(2, 0) >> mat(3, 0)
			>> mat(0, 1) >> mat(1, 1) >> mat(2, 1) >> mat(3, 1)
			>> mat(0, 2) >> mat(1, 2) >> mat(2, 2) >> mat(3, 2) >> flag;

		Matrix4f inv_mat;
		mat.inv(inv_mat);

		allCameraPoses.push_back(inv_mat);
		allFlags.push_back(flag);
	}

	in.close();
}

const std::vector<Matrix4f>& DataEngine::getAllCameraPoses() const {
	return allCameraPoses;
}

const std::vector<int>& DataEngine::getAllFlags() const {
	return allFlags;
}

const Matrix4f DataEngine::getCameraPoses(int i) const {
	return allCameraPoses[i];
}