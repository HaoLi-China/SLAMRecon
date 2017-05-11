// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#ifndef _RECONTOOLVIEW_H
#define _RECONTOOLVIEW_H

#include <QGraphicsObject>
#include <QVector3D>
#include <QMatrix4x4>
#include <QGraphicsSceneMouseEvent>
#include <QKeyEvent>
#include <QPainter>
#include "FusionEngine.h"

namespace Eigen{ class Camera; class Trackball; class Plane; }
using namespace FE;

//view to draw input images and rendering images of fusion
class ReconToolView : public QGraphicsObject
{
    Q_OBJECT
    Q_PROPERTY(QRectF rect READ boundingRect WRITE setRect)

public:
	ReconToolView(QGraphicsItem *parent);
    ~ReconToolView();

    void paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget = 0);
    void prePaint(QPainter * painter, QWidget * widget);
    void postPaint(QPainter * painter, QWidget * widget);

    QRectF rect;
    QRectF boundingRect() const { return this->rect; }
    void setRect(const QRectF & newRect){ this->rect = newRect; }

	void setFusionEngine(FusionEngine *fusionEngine);
	void setIfRendering(const bool &ifRendering);

public slots:

protected:
	unsigned int textureId[4];

public:
    // Options
    QVariantMap options;

protected:
    void mouseMoveEvent(QGraphicsSceneMouseEvent *);
    void mousePressEvent(QGraphicsSceneMouseEvent *);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *);
    void wheelEvent(QGraphicsSceneWheelEvent *);
    QPointF buttonDownCursorPos, mouseMoveCursorPos, buttonUpCursorPos;
    bool leftButtonDown, rightButtonDown, middleButtonDown;

    void keyPressEvent(QKeyEvent * event);

	FusionEngine *fusionEngine;
	UChar4Image *renderImage;
	UChar4Image *free_renderImage;
	UChar4Image *depthImage;
	UChar4Image *rgbImage;

private:
	bool ifRendering;
};

#endif //_RECONTOOLVIEW_H