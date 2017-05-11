// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#include "FileReaderEngine.h"

#include <iostream>
#include <fstream>

using namespace std;

FileReaderEngine::FileReaderEngine(const std::string rgbPath, const std::string depthPath, const std::string assoFilePath, const int frame_width, const int frame_height) :DataEngine(){
	image_height = frame_height;
	image_width = frame_width;

	m_sRgbPath = rgbPath;
	m_sDepthPath = depthPath;

	getFiles(assoFilePath);

	/*rgbFileLists = scanDirectory(rgbPath, "png");
	depthFileLists = scanDirectory(depthPath, "png");*/

	if (rgbFileLists.size() != depthFileLists.size())
		cerr << "The number of rgb and depth images is not the same!!!" << endl;
	else
		image_num = rgbFileLists.size();

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

FileReaderEngine::~FileReaderEngine(){
	if (rgbImage != NULL){
		rgbImage->Free();
	}

	if (rawDepthImage != NULL){
		rawDepthImage->Free();
	}

	if (rgbImagesBlock != NULL){
		rgbImagesBlock->Free();
	}

	if (depthImagesBlock != NULL){
		depthImagesBlock->Free();
	}
	rgbFileLists.clear();
	depthFileLists.clear();
}


bool FileReaderEngine::hasMoreImages(void){
	if (curFrameId == image_num-1)
		return false;
	else
		return true;
}

bool FileReaderEngine::getNewImages(){
	Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
	short *depth = rawDepthImage->GetData(MEMORYDEVICE_CPU);

	std::string rgb_file = m_sRgbPath + "/" + rgbFileLists.at(curFrameId + 1);
	std::string depth_file = m_sDepthPath + "/" + depthFileLists.at(curFrameId + 1);
	matRgbImage = imread(rgb_file);
	matDepthImage = imread(depth_file, -1);

	if ((matRgbImage.cols != matDepthImage.cols) || (matRgbImage.rows != matDepthImage.rows) || (matRgbImage.cols != image_width) || (matRgbImage.rows != image_height)){
		cout << endl << "The RGB and the depth frames don't have the same size.";
		return false;
	}
	else
	{
		for (int yc = 0; yc < image_height; ++yc)
		{
			for (int xc = 0; xc < image_width; ++xc)
			{
				int ind = yc*image_width + xc;
				depth[ind] = (matDepthImage.ptr<ushort>(yc)[xc])/5;
				rgb[ind][0] = matRgbImage.ptr<uchar>(yc)[xc * 3 + 2];
				rgb[ind][1] = matRgbImage.ptr<uchar>(yc)[xc * 3 + 1];
				rgb[ind][2] = matRgbImage.ptr<uchar>(yc)[xc * 3];
				rgb[ind][3] = 255;
			}
		}
	}
	curFrameId++;
#ifdef USE_IMAGES_BLOCK
	//rgbImagesBlock->saveImageToBlock(curFrameId, rgbImage);
	depthImagesBlock->saveImageToBlock(curFrameId, rawDepthImage);
#endif

	return true;
}

void FileReaderEngine::getFiles(const string assoFilePath) {
	ifstream fAssociation;
	fAssociation.open(assoFilePath.c_str());

	if (!fAssociation.is_open()){
		std::cout << "Fail to open file " << assoFilePath << std::endl;
		return;
	}

	while (!fAssociation.eof()) {
		string s;
		getline(fAssociation, s);

		if (!s.empty()) {
			stringstream ss;
			ss << s;
			double t;
			string sRGB, sD;
			ss >> t;
			ss >> sRGB;
			rgbFileLists.push_back(sRGB);
			ss >> t;
			ss >> sD;
			depthFileLists.push_back(sD);

		}
	}
}
std::vector<string> FileReaderEngine::scanDirectory(const string path, const string extension) {
	std::vector<string> fileLists;
	string dir_name = path + "/*." + extension;

	/*_finddata_t file;
	int k;
	long HANDLE;
	k = HANDLE = _findfirst(dir_name.data(), &file);
	while (k != -1)
	{
	cout << file.name << endl;
	k = _findnext(HANDLE, &file);
	}
	_findclose(HANDLE);*/

	HANDLE hSearch;
	WIN32_FIND_DATA data;
	hSearch = FindFirstFile(dir_name.data(), &data);
	//cout << data.cFileName << endl;
	fileLists.push_back(path + "/" + data.cFileName);
	while (FindNextFile(hSearch, &data)) {
		//cout << data.cFileName << endl;
		fileLists.push_back(path + "/" + data.cFileName);
	}
	FindClose(hSearch);
	return fileLists;
}