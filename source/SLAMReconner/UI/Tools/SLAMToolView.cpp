// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#include "SLAMToolView.h"

#include "../GraphicsView.h"
#include "../GraphicsScene.h"
#include "../Viewer.h"
#include "../Camera.h"

#include <QSettings>
#include <QGraphicsProxyWidget>
#include <QVBoxLayout>
#include <QSlider>
#include <QPushButton>

using namespace cv;
using namespace std;

Q_DECLARE_METATYPE(Eigen::Vector3f)

SLAMToolView::SLAMToolView(QGraphicsItem * parent)
	: QGraphicsObject(parent)
{
    // Enable keyboard
    this->setFlags(QGraphicsItem::ItemIsFocusable);
	 
    // Create camera & trackball
    {
        camera = new Eigen::Camera();
        trackball = new Eigen::Trackball();

        // Camera target and initial position
        auto frame = camera->frame();
        frame.position = Eigen::Vector3f(-1, 0, 0.5);
        camera->setTarget(Eigen::Vector3f(0,0,0.5));
        camera->setFrame(frame);

        int deltaZoom = 5; // = document->extent().length() * 1.0;

        // Default view angle
        double theta1 = acos(-1) * 0.75;
        double theta2 = acos(-1) * 0.10;
        camera->rotateAroundTarget(Eigen::Quaternionf(Eigen::AngleAxisf(theta1, Eigen::Vector3f::UnitY())));
        camera->zoom(-(4+deltaZoom));
        camera->rotateAroundTarget(Eigen::Quaternionf(Eigen::AngleAxisf(theta2, Eigen::Vector3f::UnitX())));

        trackball->setCamera(camera);
    }

    // Background:
    QSettings s;
    options["lightBackColor"].setValue(s.value("lightBackColor").value<QColor>());
    options["darkBackColor"].setValue(s.value("darkBackColor").value<QColor>());

//	options["lightBackColor"].setValue(QColor(255,255,255));
//	options["darkBackColor"].setValue(QColor(255,255,255));
	m_pMap = NULL;
	m_pCoGraph = NULL;
	m_pSpanTree = NULL;
}

SLAMToolView::~SLAMToolView()
{
    delete camera; delete trackball;
}


void SLAMToolView::setMGT(Map* pMap, CovisibilityGraph *pCograph, SpanningTree* pSpantree) {
	m_pMap = pMap;
	m_pCoGraph = pCograph;
	m_pSpanTree = pSpantree;
}

QVector<QVector3D> SLAMToolView::getCameraLines(cv::Mat pose, float boxw) {

	QVector<QVector3D> cameras;
	cv::Mat R = pose.rowRange(0, 3).colRange(0, 3);
	R = R.t();
	cv::Mat t = pose.rowRange(0, 3).col(3);
	t = -R * t;

	const float &w = boxw;
	const float h = w*0.75;
	const float z = w*0.6;

	float dvC[3] = { 0, 0, 0 };
	cv::Mat mvC(3, 1, CV_32F, dvC);
	float dvLT[3] = { -w, h, z };
	cv::Mat mvLT(3, 1, CV_32F, dvLT);
	float dvLB[3] = { -w, -h, z };
	cv::Mat mvLB(3, 1, CV_32F, dvLB);
	float dvRT[3] = { w, h, z };
	cv::Mat mvRT(3, 1, CV_32F, dvRT);
	float dvRB[3] = { w, -h, z };
	cv::Mat mvRB(3, 1, CV_32F, dvRB);

	mvC = R * mvC + t;
	mvLT = R * mvLT + t;
	mvLB = R * mvLB + t;
	mvRT = R * mvRT + t;
	mvRB = R * mvRB + t;


	

	QVector3D vC(mvC.at<float>(0), mvC.at<float>(1), mvC.at<float>(2));
	QVector3D vLT(mvLT.at<float>(0), mvLT.at<float>(1), mvLT.at<float>(2));
	QVector3D vLB(mvLB.at<float>(0), mvLB.at<float>(1), mvLB.at<float>(2));
	QVector3D vRT(mvRT.at<float>(0), mvRT.at<float>(1), mvRT.at<float>(2));
	QVector3D vRB(mvRB.at<float>(0), mvRB.at<float>(1), mvRB.at<float>(2));


	cameras << vC; cameras << vRT;
	cameras << vC; cameras << vLB;
	cameras << vC; cameras << vLT;
	cameras << vC; cameras << vRB;


	cameras << vRT; cameras << vRB;
	cameras << vLT; cameras << vLB;
	cameras << vLT; cameras << vRT;
	cameras << vLB; cameras << vRB;
	return cameras;
}

void SLAMToolView::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget * widget)
{
    prePaint(painter, widget);

    // Begin drawing 3D
    painter->beginNativePainting();

    // Get OpenGL widget
    auto glwidget = (Viewer*)widget;
    if (glwidget)
    {
        // Viewport region
        auto rect = sceneBoundingRect();

        // OpenGL draws directly to screen, compensate for that
        QPoint viewDelta = scene()->views().first()->mapFromScene(rect.topLeft());
        if (viewDelta.manhattanLength() > 5) rect.moveTopLeft(viewDelta);
        glwidget->glViewport(rect.left(), rect.top(), rect.width(), rect.height());

        // Camera settings
        QMatrix4x4 cameraMatrix;
        camera->setViewport(rect.width(), rect.height());
        Eigen::Matrix4f p = camera->projectionMatrix();
        Eigen::Matrix4f v = camera->viewMatrix().matrix();
        p.transposeInPlace();
        v.transposeInPlace();
        cameraMatrix = QMatrix4x4(p.data()) * QMatrix4x4(v.data());

        // Update local camera details
        glwidget->eyePos = QVector3D(camera->position().x(),camera->position().y(),camera->position().z());

        // Temporarily save active camera at OpenGL widget
        glwidget->pvm = cameraMatrix;

        // Grid: prepare and draw
        {
            QVector<QVector3D> lines;
            double gridWidth = 10;
            double gridSpacing = gridWidth / 30.0;
            for (GLfloat i = -gridWidth; i <= gridWidth; i += gridSpacing) {
                lines << QVector3D(i, gridWidth, 0); lines << QVector3D(i, -gridWidth, 0);
                lines << QVector3D(gridWidth, i, 0); lines << QVector3D(-gridWidth, i, 0);
            }

            // Grid color
            QColor color = QColor::fromRgbF(0.5, 0.5, 0.5, 0.1);

            // Draw grid
            glwidget->glLineWidth(1.0f);
            glwidget->drawLines(lines, color, cameraMatrix, "grid_lines");
        }

        // Draw debug shape
        // glwidget->drawBox(3, 1, 2, cameraMatrix);

		if (m_pMap != NULL) {

			

			QVector<QVector3D> points;
			float size = 3;

			std::vector<MapPoint*> vpMapPoints = m_pMap->GetAllMapPoints();
			for (std::vector<MapPoint*>::iterator vit = vpMapPoints.begin(), vend = vpMapPoints.end(); vit != vend; vit++) {
				MapPoint* mappoint = *vit;
				cv::Mat p = mappoint->GetWorldPos();
				QVector3D point(p.at<float>(0), p.at<float>(1), p.at<float>(2));
				points.append(point);
			}
			glwidget->drawPoints(points, size, QColor(255, 0, 0), cameraMatrix);

			// 绘制当前Frame
			cv::Mat curPose = m_pMap->getCurFramePose();
			if (!curPose.empty()) {
				QVector<QVector3D> curCameras = getCameraLines(curPose, 0.06);

				glwidget->glLineWidth(1.5f);
				glwidget->drawLines(curCameras, QColor(0, 255, 0), cameraMatrix, "lines");
			}


			QVector<QVector3D> graphes;
			std::vector<KeyFrame*> vKeyFrames = m_pMap->GetAllKeyFrames();

			for (std::vector<KeyFrame*>::iterator vit = vKeyFrames.begin(), vend = vKeyFrames.end(); vit != vend; vit++) {

				

				KeyFrame *frame = *vit;

				cv::Mat pose = frame->m_Transformation.clone();
				QVector<QVector3D> cameras = getCameraLines(pose, 0.05);

				glwidget->glLineWidth(1.0f);
				glwidget->drawLines(cameras, QColor(0, 0, 255), cameraMatrix, "lines");





				// 把每一帧对应的图片绘制上
				QVector<QVector3D> plane;

				//plane << vRB;
				//plane << vLB;
				//plane << vLT;
				//plane << vRT;

				plane << cameras[7];
				plane << cameras[3];
				plane << cameras[5];
				plane << cameras[1];

				const uchar *pSrc = (const uchar*)frame->m_rgbImg.data;
				QImage image(pSrc, frame->m_rgbImg.cols, frame->m_rgbImg.rows, frame->m_rgbImg.step, QImage::Format_RGB888);
				QImage img2 = image.scaled(160, 120, Qt::KeepAspectRatio);
				glwidget->drawTexturedPlane(img2.rgbSwapped(), plane, cameraMatrix);



				// 下面是为每一个Frame添加Essential Graph的过程
				const vector<KeyFrame*> vCovKFs = m_pCoGraph->GetCovisiblesByWeight(frame, 70);

				cv::Mat Ow = frame->GetCameraCenter();
				if (!vCovKFs.empty()) {
					for (vector<KeyFrame*>::const_iterator vit = vCovKFs.begin(), vend = vCovKFs.end(); vit != vend; vit++) {
						if ((*vit)->m_nKFId < frame->m_nKFId)
							continue;
						cv::Mat Ow2 = (*vit)->GetCameraCenter();
						QVector3D point1(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
						QVector3D point2(Ow2.at<float>(0), Ow2.at<float>(1), Ow2.at<float>(2));
						graphes << point1; graphes << point2;
					}
				}

				// Spanning tree
				KeyFrame* pParent = m_pSpanTree->GetParent(frame);
				if (pParent) {
					cv::Mat Owp = pParent->GetCameraCenter();
					QVector3D point1(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
					QVector3D point2(Owp.at<float>(0), Owp.at<float>(1), Owp.at<float>(2));
					graphes << point1; graphes << point2;
				}

				// Loops
				set<KeyFrame*> sLoopKFs = m_pSpanTree->GetLoopEdges(frame);
				for (set<KeyFrame*>::iterator sit = sLoopKFs.begin(), send = sLoopKFs.end(); sit != send; sit++) {
					if ((*sit)->m_nKFId < frame->m_nKFId)
						continue;
					cv::Mat Owl = (*sit)->GetCameraCenter();
					QVector3D point1(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
					QVector3D point2(Owl.at<float>(0), Owl.at<float>(1), Owl.at<float>(2));
					graphes << point1; graphes << point2;
				}
			}
			glwidget->drawLines(graphes, QColor(0, 255, 0), cameraMatrix, "lines");
		}
    }

	

    // End 3D drawing
    painter->endNativePainting();

    postPaint(painter, widget);
}

void SLAMToolView::prePaint(QPainter *painter, QWidget *)
{
    // Background
	auto lightBackColor = options["lightBackColor"].value<QColor>();
	auto darkBackColor = options["darkBackColor"].value<QColor>();

	QLinearGradient gradient(0, 0, 0, rect.height());
	gradient.setColorAt(0.0, lightBackColor);
	gradient.setColorAt(1.0, darkBackColor);
	painter->fillRect(rect, gradient);
}

void SLAMToolView::postPaint(QPainter *painter, QWidget *widget)
{
    // Border
	/*
	painter->setPen(QPen(Qt::gray, 10));
	painter->drawRect(rect);
	*/
	
	
	
	//ff->paint(painter, NULL, widget);

}

void SLAMToolView::mouseMoveEvent(QGraphicsSceneMouseEvent * event)
{
    mouseMoveCursorPos = event->pos();

    if(rightButtonDown)
    {
        trackball->track(Eigen::Vector2i(mouseMoveCursorPos.x(), mouseMoveCursorPos.y()));
    }

    if(middleButtonDown)
    {
        auto delta = mouseMoveCursorPos - buttonDownCursorPos;

        Eigen::Vector3f up = camera->up() * 0.001f * delta.y();
        Eigen::Vector3f right = camera->right() * 0.001f * delta.x();
        Eigen::Vector3f d = up + right;

        camera->setPosition(property("cameraPos").value<Eigen::Vector3f>() - d);
    }

    scene()->update(sceneBoundingRect());
}

void SLAMToolView::mousePressEvent(QGraphicsSceneMouseEvent * event)
{
    // Record mouse states
    buttonDownCursorPos = event->pos();
    mouseMoveCursorPos = buttonDownCursorPos;
	leftButtonDown = event->buttons() & Qt::LeftButton;
	rightButtonDown = event->buttons() & Qt::RightButton;
	middleButtonDown = event->buttons() & Qt::MiddleButton;

    // Camera movement
    if(rightButtonDown) {
        trackball->start(Eigen::Trackball::Around);
        trackball->track(Eigen::Vector2i(buttonDownCursorPos.x(), buttonDownCursorPos.y()));
    }

	if (leftButtonDown) {
		// Convert click to ray
		int screenWidth = rect.width();
		int screenHeight = rect.height();
		camera->setViewport(screenWidth, screenHeight);
		Eigen::Vector3f orig = camera->position();
		Eigen::Vector3f dir = camera->unProject(Eigen::Vector2f(event->pos().x(), screenHeight - event->pos().y()), 10) - orig;
		dir.normalize();
		auto rayOrigin = QVector3D(orig[0],orig[1],orig[2]);
		auto rayDirection = QVector3D(dir[0], dir[1], dir[2]);

		// Select part
		/*
		cout << orig[0] << " " << orig[1] << " " << orig[2] << endl;
		cout << dir[0] << " " << dir[1] << " " << dir[2] << endl;*/
	}

    if(middleButtonDown) {
        setProperty("cameraPos", QVariant::fromValue(camera->position()));
    }

    scene()->update(sceneBoundingRect());
}

void SLAMToolView::mouseReleaseEvent(QGraphicsSceneMouseEvent *)
{
    this->setFocus();

	leftButtonDown = false;
	rightButtonDown = false;
	middleButtonDown = false;

    scene()->update(sceneBoundingRect());
}

void SLAMToolView::wheelEvent(QGraphicsSceneWheelEvent * event)
{
    QGraphicsObject::wheelEvent(event);
    double step = (double)event->delta() / 120;
    camera->zoom(step);

    scene()->update(sceneBoundingRect());
}

void SLAMToolView::keyPressEvent(QKeyEvent *){

}
