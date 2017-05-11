// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#include "SlamReconManager.h"

#include <QFileDialog>
#include <thread>

using namespace cv;

SlamReconManager::SlamReconManager() : m_bInit(false), m_ShutdowmFlag(false), m_FusionFlag(false)
{
	srkPtr = NULL;

	deviceFlag = KitsInfo::Device::KINECT1;
	methodFlag = KitsInfo::Method::SLAMRECON;

	stopFlag = false;
	resetFlag = false;
	allDone = true;
}

SlamReconManager::~SlamReconManager()
{

}

void SlamReconManager::chooseKinectOne()
{
	deviceFlag = KitsInfo::Device::KINECT1;
}

void SlamReconManager::chooseFiles()
{
	deviceFlag = KitsInfo::Device::FILES;
}

void SlamReconManager::chooseSLAMRecon()
{
	methodFlag = KitsInfo::Method::SLAMRECON;
}

void SlamReconManager::chooseKinectFusion()
{
	methodFlag = KitsInfo::Method::KINFU;
}

bool SlamReconManager::initSystem()
{
	srkPtr = SlamReconKits::Ptr(new SlamReconKits());
	m_bInit = srkPtr->initKits(deviceFlag, methodFlag);

	return m_bInit;
}

bool SlamReconManager::startSystem()
{
	if (stopFlag == true){
		stopFlag = false;
		return true;
	}

	switch (methodFlag){
	case KitsInfo::Method::SLAMRECON:
		return doSlamRecon();

	case KitsInfo::Method::KINFU:
		return doKinectFusion();
	}
}

void SlamReconManager::kinectFusionProcess()
{
	DataEngine::Ptr dataEngine = srkPtr->dataEnginePtr;
	FusionEngine *fusionEngine = srkPtr->fusionCompoPtr->fusionEngine;

	if (dataEngine == NULL || fusionEngine == NULL){
		std::cerr << "failed!" << std::endl;
		return;
	}

	allDone = false;

	int currentFrameNo = 0;
	UChar4Image *inputRGBImage;
	ShortImage *inputRawDepthImage;

	while (dataEngine->hasMoreImages() && !resetFlag){
		if (!stopFlag){
			dataEngine->getNewImages();
			inputRGBImage = dataEngine->getCurrentRgbImage();
			inputRawDepthImage = dataEngine->getCurrentDepthImage();

			//actual processing on the mainEngine
			fusionEngine->ProcessFrame(inputRGBImage, inputRawDepthImage);

			FESafeCall(cudaThreadSynchronize());

			emit updateFusionView();

			std::cout << "currentFrameNo: " << currentFrameNo << std::endl;
			currentFrameNo++;
		}
	}

	allDone = true;
}

bool SlamReconManager::doKinectFusion()
{
	if (!m_bInit){
		cout << "The System need to be initialized!!! " << endl;
		return false;
	}

	std::thread* ThreadProcess = new std::thread(&SlamReconManager::kinectFusionProcess, this);

	return true;
}

void SlamReconManager::fusionProcess() {
	DataEngine::Ptr dataEngine = srkPtr->dataEnginePtr;
	FusionEngine *fusionEngine = srkPtr->fusionCompoPtr->fusionEngine;
	Map* m_pMap = srkPtr->slamCompoPtr->m_pMap;
	SpanningTree* m_pSpanTree = srkPtr->slamCompoPtr->m_pSpanTree;

	if (dataEngine == NULL || fusionEngine == NULL || m_pMap == NULL || m_pSpanTree == NULL){
		std::cerr << "failed!" << std::endl;
		return;
	}

	Sleep(5);

	ShortImagesBlock* depthBlock = dataEngine->getDepthImagesBlock();
	pair<int, Mat> fIdAndPose;

	fIdAndPose = m_pMap->getIdAndPose();
	int flag = fIdAndPose.first;

	UChar4Image *inputRGBImage = dataEngine->getCurrentRgbImage();
	ShortImage *inputRawDepthImage = new ShortImage(dataEngine->getDepthImageSize(), true, true);

	cout << endl << "fusionThread " << endl;

	while (!resetFlag) {
		if (flag == -1){
			KeyFrame *pKF = m_pMap->getModifiedKeyFrame();

			if (pKF != NULL){
				cout << "Refusion KeyFrame Id: " << pKF->m_nKFId << " it's frame is " << pKF->m_nFId << endl;

				Mat pose = Mat::eye(4, 4, CV_32F);
				Mat oldpose = Mat::eye(4, 4, CV_32F);

				while (pKF->isBad()) {
					pose = pose*pKF->m_Tcp;
					oldpose = oldpose*pKF->m_Tcp;
					pKF = m_pSpanTree->GetParent(pKF);
				}

				pose = pose*pKF->GetPose();
				oldpose = oldpose*pKF->m_oldCameraPose;


				Mat errorpose = pose - oldpose;
				double error = errorpose.dot(errorpose);

				if (error > 0.001) {
					list<pair<int, Mat> > lIdPoses = m_pMap->getFramesByKF(pKF, m_pSpanTree);

					for (list<pair<int, Mat> >::iterator lit = lIdPoses.begin(), lend = lIdPoses.end(); lit != lend; lit++) {
						pair<int, Mat> modifyFIdAndPose = *lit;

						depthBlock->readImageToCpu(modifyFIdAndPose.first, inputRawDepthImage);

						cout << "Refusion frame: " << modifyFIdAndPose.first << endl;

						Mat framePose = modifyFIdAndPose.second * pose;
						Mat frameOldPose = modifyFIdAndPose.second * oldpose;

						Matrix4f newMt(framePose.at<float>(0, 0), framePose.at<float>(1, 0), framePose.at<float>(2, 0), framePose.at<float>(3, 0),
							framePose.at<float>(0, 1), framePose.at<float>(1, 1), framePose.at<float>(2, 1), framePose.at<float>(3, 1),
							framePose.at<float>(0, 2), framePose.at<float>(1, 2), framePose.at<float>(2, 2), framePose.at<float>(3, 2),
							framePose.at<float>(0, 3), framePose.at<float>(1, 3), framePose.at<float>(2, 3), framePose.at<float>(3, 3));

						Matrix4f oldMt(frameOldPose.at<float>(0, 0), frameOldPose.at<float>(1, 0), frameOldPose.at<float>(2, 0), frameOldPose.at<float>(3, 0),
							frameOldPose.at<float>(0, 1), frameOldPose.at<float>(1, 1), frameOldPose.at<float>(2, 1), frameOldPose.at<float>(3, 1),
							frameOldPose.at<float>(0, 2), frameOldPose.at<float>(1, 2), frameOldPose.at<float>(2, 2), frameOldPose.at<float>(3, 2),
							frameOldPose.at<float>(0, 3), frameOldPose.at<float>(1, 3), frameOldPose.at<float>(2, 3), frameOldPose.at<float>(3, 3));

						fusionEngine->ReprocessFrame(inputRGBImage, inputRawDepthImage, modifyFIdAndPose.first, oldMt, newMt);

					}
					pKF->m_oldCameraPose = pKF->GetPose();
				}

				fIdAndPose = m_pMap->getIdAndPose();
				flag = fIdAndPose.first;

				continue;
			}
			else{
				if (m_ShutdowmFlag){
					cout << "The fusion is down!!!" << endl;
					break;
				}
				else {
					fIdAndPose = m_pMap->getIdAndPose();
					flag = fIdAndPose.first;
					continue;
				}
			}
		}

		cout << "first fusion frame: " << fIdAndPose.first << endl;

		Mat pose = fIdAndPose.second;

		if (pose.empty())
			continue;

		Matrix4f mt(pose.at<float>(0, 0), pose.at<float>(1, 0), pose.at<float>(2, 0), pose.at<float>(3, 0),
			pose.at<float>(0, 1), pose.at<float>(1, 1), pose.at<float>(2, 1), pose.at<float>(3, 1),
			pose.at<float>(0, 2), pose.at<float>(1, 2), pose.at<float>(2, 2), pose.at<float>(3, 2),
			pose.at<float>(0, 3), pose.at<float>(1, 3), pose.at<float>(2, 3), pose.at<float>(3, 3));

		{
			depthBlock->readImageToCpu(fIdAndPose.first, inputRawDepthImage);
		}

		fusionEngine->ProcessFrame(inputRGBImage, inputRawDepthImage, fIdAndPose.first, mt);

		FESafeCall(cudaThreadSynchronize());

		emit updateFusionView();

		fIdAndPose = m_pMap->getIdAndPose();
		flag = fIdAndPose.first;

		Sleep(1);

	}

	m_FusionFlag = true;
}

void SlamReconManager::slamReconProcess()
{
	if (!m_bInit){
		std::cerr << "The System need to be initialized!!! " << std::endl;
		return;
	}

	DataEngine::Ptr dataEngine = srkPtr->dataEnginePtr;
	SLAM *slamEngine = srkPtr->slamCompoPtr->slamEngine;
	
	if (dataEngine == NULL || slamEngine == NULL){
		std::cerr << "failed!" << std::endl;
		return;
	}

	allDone = false;

	ShortImagesBlock* depthBlock = dataEngine->getDepthImagesBlock();

	std::thread* ThreadProcess = new std::thread(&SlamReconManager::fusionProcess, this);

	Mat rgbImg;
	Mat depthImg;
	UChar4Image *inputRGBImage;
	ShortImage *inputRawDepthImage;

	int currentFrameNo = 0;
	while (dataEngine->hasMoreImages() && !resetFlag){
		if (!stopFlag){
			dataEngine->getNewImages();

			rgbImg = dataEngine->getCurrentMatRgbImage();
			depthImg = dataEngine->getCurrentMatDepthImage();

			slamEngine->trackRGBD(rgbImg, depthImg, 0);

			std::cout << "currentFrameNo: " << currentFrameNo << std::endl;

			currentFrameNo++;
			emit updateSLAMView();
		}
	}

	slamEngine->Shutdown(m_ShutdowmFlag);

	cout << "m_ShutdowmFlag is" << m_ShutdowmFlag << endl;

	while (!m_FusionFlag)
		Sleep(5);

	allDone = true;
	cout << "The Slam and fusion is down!!!!" << endl;
}

bool SlamReconManager::doSlamRecon()
{
	if (!m_bInit){
		cout << "The System need to be initialized!!! " << endl;
		return false;
	}
	std::thread* ThreadProcess = new std::thread(&SlamReconManager::slamReconProcess, this);

	return true;
}

void SlamReconManager::saveMesh(){
	stopSystem();

	switch (methodFlag){
	case KitsInfo::Method::SLAMRECON:
		saveMeshForSLAMRecon();
		break;
	case KitsInfo::Method::KINFU:
		saveMeshForKinfu();
		break;
	}
}

void SlamReconManager::saveMeshForSLAMRecon()
{
	DataEngine::Ptr dataEngine = srkPtr->dataEnginePtr;
	FusionEngine *fusionEngine = srkPtr->fusionCompoPtr->fusionEngine;
	Map* m_pMap = srkPtr->slamCompoPtr->m_pMap;
	SpanningTree* m_pSpanTree = srkPtr->slamCompoPtr->m_pSpanTree;

	if (dataEngine == NULL || fusionEngine == NULL || m_pMap == NULL || m_pSpanTree == NULL){
		return;
	}


	QString dataPath = "../../data/";
	QString filename = QFileDialog::getSaveFileName(0, tr("Save mesh"), dataPath, tr("Text Files (*.stl)"));
	if (filename.isEmpty()) return;

	//get all camera pose
	vector<Matrix4f> allCameraPoses;

	vector<KeyFrame*> vpKFs = m_pMap->GetAllKeyFrames();
	sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);
	Mat Two = vpKFs[0]->GetPoseInverse();
	list<KeyFrame*>::iterator lRit = m_pMap->m_lpReferences.begin();

	list<bool>::iterator lbKF = m_pMap->m_lbKF.begin();
	for (list<Mat>::iterator lit = m_pMap->m_lRelativeFramePoses.begin(), lend = m_pMap->m_lRelativeFramePoses.end(); lit != lend; lit++, lRit++, lbKF++) {
		bool bKF = *lbKF;
		KeyFrame* pKF = *lRit;

		Mat Trw = Mat::eye(4, 4, CV_32F);

		while (pKF->isBad()) {
			//cout << "bad KeyFrame" << pKF->m_nKFId << endl;
			Trw = Trw*pKF->m_Tcp;
			pKF = m_pSpanTree->GetParent(pKF);
			bKF = false;

		}

		Trw = Trw*pKF->GetPose()*Two;

		Mat Tcw = (*lit)*Trw;
		Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
		Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);

		Matrix4f mat(Rwc.at<float>(0, 0), Rwc.at<float>(1, 0), Rwc.at<float>(2, 0), 0,
			Rwc.at<float>(0, 1), Rwc.at<float>(1, 1), Rwc.at<float>(2, 1), 0,
			Rwc.at<float>(0, 2), Rwc.at<float>(1, 2), Rwc.at<float>(2, 2), 0,
			twc.at<float>(0), twc.at<float>(1), twc.at<float>(2), 1);

		Matrix4f inv_mat;
		mat.inv(inv_mat);

		allCameraPoses.push_back(inv_mat);
	}

	//fusion
	UChar4Image *inputRGBImage = dataEngine->getCurrentRgbImage();
	ShortImage *inputRawDepthImage = dataEngine->getCurrentDepthImage();

	ShortImagesBlock* depthBlock = dataEngine->getDepthImagesBlock();

	if (allCameraPoses.size() != dataEngine->getCurrentFrameId() + 1){
		return;
	}

	fusionEngine->resetScene();

	for (int i = 0; i <= dataEngine->getCurrentFrameId(); i++){
		depthBlock->readImageToCpu(i, inputRawDepthImage);
		fusionEngine->ProcessFrame(inputRGBImage, inputRawDepthImage, i, allCameraPoses[i]);

		FESafeCall(cudaThreadSynchronize());
		std::cout << "currentFrameNo: " << i << std::endl;
	}

	fusionEngine->SaveSceneToMesh(filename.toStdString().c_str());

	cout << "Done!" << endl;
}

void SlamReconManager::saveMeshForKinfu()
{
	FusionEngine *fusionEngine = srkPtr->fusionCompoPtr->fusionEngine;

	if (fusionEngine == NULL){
		return;
	}

	QString dataPath = "../../data/";
	QString filename = QFileDialog::getSaveFileName(0, tr("Save mesh"), dataPath, tr("Text Files (*.stl)"));
	if (filename.isEmpty()) return;

	fusionEngine->SaveSceneToMesh(filename.toStdString().c_str());

	cout << "Done!" << endl;
}

void SlamReconManager::stopSystem()
{
	stopFlag = true;
}

void SlamReconManager::resetSystem()
{
	resetFlag = true;
	
	while (!allDone){
		Sleep(100);
	}

	resetFlag = false;
	stopFlag = false;
	//srkPtr = NULL;
	srkPtr.reset();
	m_bInit = false;
	m_ShutdowmFlag = false; 
	m_FusionFlag = false;

	emit updateSLAMView();
	emit updateFusionView();
}

int SlamReconManager::getDeviceFlag()
{
	return deviceFlag;
}

int SlamReconManager::getMethodFlag()
{
	return methodFlag;
}
