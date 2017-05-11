// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#include "ReconToolView.h"

#include "../GraphicsView.h"
#include "../GraphicsScene.h"
#include "../Viewer.h"
#include "../Camera.h"
#include "Utility.h"

#include <QSettings>
#include <QGraphicsProxyWidget>
#include <QVBoxLayout>
#include <QSlider>
#include <QPushButton>

#include <vector>

Q_DECLARE_METATYPE(Eigen::Vector3f)

ReconToolView::ReconToolView(QGraphicsItem * parent)
: QGraphicsObject(parent)
{
	glGenTextures(4, textureId);

	fusionEngine = NULL;

	renderImage = NULL;
	free_renderImage = NULL;
	depthImage = NULL;
	rgbImage = NULL;

	// Background:
	QSettings s;
	options["lightBackColor"].setValue(s.value("lightBackColor").value<QColor>());
	options["darkBackColor"].setValue(s.value("darkBackColor").value<QColor>());

	ifRendering = false;

	//options["lightBackColor"].setValue(QColor(255, 0, 0));
	//options["darkBackColor"].setValue(QColor(255, 0, 0));
}

ReconToolView::~ReconToolView()
{
	if (renderImage != NULL){
		renderImage->Free();
	}

	if (free_renderImage != NULL){
		free_renderImage->Free();
	}

	if (depthImage != NULL){
		depthImage->Free();
	}

	if (rgbImage != NULL){
		rgbImage->Free();
	}
}

void ReconToolView::setFusionEngine(FusionEngine *fusionEngine){
	this->fusionEngine = fusionEngine;
}

void ReconToolView::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget * widget)
{
	if (fusionEngine == NULL || !ifRendering){
		return;
	}

	if (free_renderImage == NULL){
		free_renderImage = new UChar4Image(fusionEngine->GetImageSize(), true, false);
	}

	if (renderImage == NULL){
		renderImage = new UChar4Image(fusionEngine->GetImageSize(), true, false);
	}

	if (depthImage == NULL){
		depthImage = new UChar4Image(fusionEngine->GetImageSize(), true, false);
	}

	if (rgbImage == NULL){
		rgbImage = new UChar4Image(fusionEngine->GetImageSize(), true, false);
	}

	prePaint(painter, widget);
	// Begin drawing 3D
	painter->beginNativePainting();

	auto glwidget = (Viewer*)widget;

	if (glwidget){
		// Viewport region
		auto rect = sceneBoundingRect();
		QPoint viewDelta = scene()->views().first()->mapFromScene(rect.topLeft());
		if (viewDelta.manhattanLength() > 5) rect.moveTopLeft(viewDelta);
		glwidget->glViewport(rect.left(), rect.top(), rect.width(), rect.height());

		fusionEngine->GetImage(renderImage, FusionEngine::IMAGE_SCENERAYCAST);
		fusionEngine->GetImage(free_renderImage, FusionEngine::IMAGE_FREECAMERA_CAST);
		fusionEngine->GetImage(depthImage, FusionEngine::IMAGE_ORIGINAL_DEPTH);
		fusionEngine->GetImage(rgbImage, FusionEngine::IMAGE_ORIGINAL_RGB);

		glwidget->drawRenderingImgs(free_renderImage->GetData(MEMORYDEVICE_CPU), free_renderImage->noDims, textureId[0],
			renderImage->GetData(MEMORYDEVICE_CPU), renderImage->noDims, textureId[1],
			rgbImage->GetData(MEMORYDEVICE_CPU), rgbImage->noDims, textureId[2],
			depthImage->GetData(MEMORYDEVICE_CPU), depthImage->noDims, textureId[3]);
	}

	painter->endNativePainting();

	postPaint(painter, widget);
}

void ReconToolView::prePaint(QPainter *painter, QWidget *)
{
	// Background
	auto lightBackColor = options["lightBackColor"].value<QColor>();
	auto darkBackColor = options["darkBackColor"].value<QColor>();

	QLinearGradient gradient(0, 0, 0, rect.height());
	gradient.setColorAt(0.0, lightBackColor);
	gradient.setColorAt(1.0, darkBackColor);
	painter->fillRect(rect, gradient);
}

void ReconToolView::postPaint(QPainter *, QWidget *)
{
	// Border
	//painter->setPen(QPen(Qt::gray, 10));
	//painter->drawRect(rect);
}

void ReconToolView::mouseMoveEvent(QGraphicsSceneMouseEvent * event)
{
	if (fusionEngine == NULL || !ifRendering){
		return;
	}

	mouseMoveCursorPos = event->pos();
	auto delta = mouseMoveCursorPos - buttonDownCursorPos;

	buttonDownCursorPos = mouseMoveCursorPos;

	Vector2i movement;
	movement.x = delta.x();
	movement.y = delta.y();

	if ((movement.x == 0) && (movement.y == 0)) return;

	static const float scale_rotation = 0.005f;
	static const float scale_translation = 0.0025f;

	if (leftButtonDown)
	{
		Vector3f axis((float)-movement.y, (float)-movement.x, 0.0f);
		float angle = scale_rotation * sqrt((float)(movement.x * movement.x + movement.y*movement.y));
		Matrix3f rot = Basis::Utility::createRotation(axis, angle);

		FEPose pose = fusionEngine->getFreePose();
		pose.SetRT(rot * pose.GetR(), rot * pose.GetT());
		fusionEngine->setFreePose(pose);
	}

	if (rightButtonDown)
	{
		// right button: translation in x and y direction
		FEPose pose = fusionEngine->getFreePose();
		pose.SetT(pose.GetT() + scale_translation * Vector3f((float)movement.x, (float)movement.y, 0.0f));
		fusionEngine->setFreePose(pose);
	}

	if (middleButtonDown)
	{
		// middle button: translation along z axis
		FEPose pose = fusionEngine->getFreePose();
		pose.SetT(pose.GetT() + scale_translation * Vector3f(0.0f, 0.0f, (float)movement.y));
		fusionEngine->setFreePose(pose);
	}

	fusionEngine->renderFreeView();
	scene()->update(sceneBoundingRect());
}

void ReconToolView::mousePressEvent(QGraphicsSceneMouseEvent * event)
{
	if (fusionEngine == NULL || !ifRendering){
		return;
	}

	// Record mouse states
	buttonDownCursorPos = event->pos();
	mouseMoveCursorPos = buttonDownCursorPos;
	leftButtonDown = event->buttons() & Qt::LeftButton;
	rightButtonDown = event->buttons() & Qt::RightButton;
	middleButtonDown = event->buttons() & Qt::MiddleButton;

	scene()->update(sceneBoundingRect());
}

void ReconToolView::mouseReleaseEvent(QGraphicsSceneMouseEvent *)
{
	if (fusionEngine == NULL || !ifRendering){
		return;
	}

	this->setFocus();

	leftButtonDown = false;
	rightButtonDown = false;
	middleButtonDown = false;

	fusionEngine->renderFreeView();
	scene()->update(sceneBoundingRect());
}

void ReconToolView::wheelEvent(QGraphicsSceneWheelEvent * event)
{
	if (fusionEngine == NULL || !ifRendering){
		return;
	}

	FEPose pose = fusionEngine->getFreePose();

	static const float scale_translation = 0.05f;

	pose.SetT(pose.GetT() + scale_translation * Vector3f(0.0f, 0.0f, (event->delta() > 0) ? -1.0f : 1.0f));

	fusionEngine->setFreePose(pose);

	fusionEngine->renderFreeView();
	scene()->update(sceneBoundingRect());
}

void ReconToolView::keyPressEvent(QKeyEvent *){

}

void ReconToolView::setIfRendering(const bool &ifRendering){
	this->ifRendering = ifRendering;
}
