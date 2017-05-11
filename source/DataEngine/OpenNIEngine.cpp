// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#include <iostream>
#include "OpenNIEngine.h"

using namespace std;

OpenNIEngine::OpenNIEngine():DataEngine(){
    openCamera();

    oniDepthStream.readFrame(&oniDepthImg);
    oniColorStream.readFrame(&oniColorImg);

    image_height = oniColorImg.getHeight();
    image_width = oniColorImg.getWidth();

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

OpenNIEngine::~OpenNIEngine(){
    closeCamera();

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
}

bool OpenNIEngine::openCamera(){
    Status rc = STATUS_OK;

    //initialize OpenNI2
    rc = OpenNI::initialize();

    //open kinect or xtion
    const char * deviceURL = openni::ANY_DEVICE;
    rc = device.open(deviceURL);

    //create depth stream
    rc = oniDepthStream.create(device, SENSOR_DEPTH);
    if (STATUS_OK == rc)
    {
        //set depth video mode
        VideoMode modeDepth;
        modeDepth.setResolution(640, 480);
        modeDepth.setFps(30);
        modeDepth.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
        oniDepthStream.setVideoMode(modeDepth);
    }
    else
    {
        cerr << "Can't create depth stream: " << OpenNI::getExtendedError() << endl;
        return false;
    }

    //create color stream
    rc = oniColorStream.create(device, openni::SENSOR_COLOR);
    if (STATUS_OK == rc)
    {
        //set color video mode
        VideoMode modeColor;
        modeColor.setResolution(640, 480);
        modeColor.setFps(30);
        modeColor.setPixelFormat(PIXEL_FORMAT_RGB888);
        oniColorStream.setVideoMode(modeColor);
    }
    else
    {
        cerr << "Can't create color stream: " << OpenNI::getExtendedError() << endl;
        return false;
    }

    if (!oniDepthStream.isValid() || !oniColorStream.isValid())
    {
        cerr << "illegal!" << endl;
        OpenNI::shutdown();
        return false;
    }

    openni::Status s;
    if (device.isImageRegistrationModeSupported(IMAGE_REGISTRATION_DEPTH_TO_COLOR))
    {
        s = device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR); //depth to color registration
    }

    rc = oniDepthStream.start(); //open depth stream
    if (STATUS_OK != rc)
    {
        cerr << "Can't open depth stream: " << OpenNI::getExtendedError() << endl;
        oniDepthStream.destroy();

        return false;
    }

    rc = oniColorStream.start(); //open color stream
    if (STATUS_OK != rc)
    {
        cerr << "Can't open color stream: " << OpenNI::getExtendedError() << endl;
        oniColorStream.destroy();

        return false;
    }

    return true;
}

bool OpenNIEngine::closeCamera(){
    oniDepthStream.destroy();
    oniColorStream.destroy();
    device.close();
    OpenNI::shutdown();

    return true;
}


bool OpenNIEngine::hasMoreImages(){
    return true;
}

bool OpenNIEngine::getNewImages(){
	Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
	short *depth = rawDepthImage->GetData(MEMORYDEVICE_CPU);

    oniDepthStream.readFrame(&oniDepthImg);
    oniColorStream.readFrame(&oniColorImg);

    const int height = oniColorImg.getHeight();
    const int width = oniColorImg.getWidth();

    //    image_width = width;
    //    image_height = height;

    if ((oniDepthImg.getWidth() != oniColorImg.getWidth()) || (oniDepthImg.getHeight() != oniColorImg.getHeight())){
        cout << endl << "The RGB and the depth frames don't have the same size.";
        return false;
    }
    else
    {
        //Read new frame
        const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)oniDepthImg.getData();
        const openni::RGB888Pixel* pRgbRow = (const openni::RGB888Pixel*)oniColorImg.getData();
        int rowSize = oniColorImg.getStrideInBytes() / sizeof(openni::RGB888Pixel);

        for (int yc = 0; yc < height; yc++)
        {
            const openni::RGB888Pixel* pRgb = pRgbRow;
            const openni::DepthPixel* pDepth = pDepthRow;
            for (int xc = 0; xc < width; xc++, ++pRgb, ++pDepth)
            {
                int ind = yc*width + width -1 - xc;
				depth[ind] = *pDepth;
				rgb[ind][0] = pRgb->r;
				rgb[ind][1] = pRgb->g;
				rgb[ind][2] = pRgb->b;
				rgb[ind][3] = 255;

				matDepthImage.ptr<ushort>(yc)[xc] = depth[ind] * 5;
				matRgbImage.ptr<uchar>(yc)[xc * 3 + 2] = rgb[ind][0];
				matRgbImage.ptr<uchar>(yc)[xc * 3 + 1] = rgb[ind][1];
				matRgbImage.ptr<uchar>(yc)[xc * 3] = rgb[ind][2];
            }
            pRgbRow += rowSize;
            pDepthRow += rowSize;
        }

		for (int yc = 0; yc < height; yc++)
		{
			for (int xc = 0; xc < width; xc++)
			{
				int ind = yc*width + xc;
				matDepthImage.ptr<ushort>(yc)[xc] = depth[ind] * 5;
				matRgbImage.ptr<uchar>(yc)[xc * 3 + 2] = rgb[ind][0];
				matRgbImage.ptr<uchar>(yc)[xc * 3 + 1] = rgb[ind][1];
				matRgbImage.ptr<uchar>(yc)[xc * 3] = rgb[ind][2];
			}
			pRgbRow += rowSize;
			pDepthRow += rowSize;
		}
    }

	curFrameId++;

#ifdef USE_IMAGES_BLOCK
	//rgbImagesBlock->saveImageToBlock(curFrameId, rgbImage);
	depthImagesBlock->saveImageToBlock(curFrameId, rawDepthImage);
#endif

	return true;
}



