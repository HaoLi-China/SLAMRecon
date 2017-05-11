// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#ifndef _SLAMTOOLVIEW_H
#define _SLAMTOOLVIEW_H

#include <QGraphicsObject>
#include <QVector3D>
#include <QMatrix4x4>
#include <QGraphicsSceneMouseEvent>
#include <QKeyEvent>
#include <QPainter>
#include "../SLAMEngine/SLAM/Map.h"
#include "../SLAMEngine/SLAM/CovisibilityGraph.h"
#include "../SLAMEngine/SLAM/SpanningTree.h"

using namespace SLAMRecon;
namespace Eigen{ class Camera; class Trackball; class Plane; }

//view to draw camera poses and map points of SLAM
class SLAMToolView : public QGraphicsObject
{
    Q_OBJECT
    Q_PROPERTY(QRectF rect READ boundingRect WRITE setRect)

public:
	SLAMToolView(QGraphicsItem *parent);
    ~SLAMToolView();

    void paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget = 0);
    void prePaint(QPainter * painter, QWidget * widget);
    void postPaint(QPainter * painter, QWidget * widget);

    QRectF rect;
    QRectF boundingRect() const { return this->rect; }
    void setRect(const QRectF & newRect){ this->rect = newRect; }


	QVector<QVector3D> getCameraLines(cv::Mat pose, float boxw);

public:
	void setMGT(Map* pMap, CovisibilityGraph *pCograph, SpanningTree* pSpantree);

public slots:

protected:
    // Camera movement
    Eigen::Camera* camera;
    Eigen::Trackball* trackball;
	
	Map* m_pMap;
	SpanningTree* m_pSpanTree;
	CovisibilityGraph* m_pCoGraph;

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

signals:

};

#endif //_SLAMTOOLVIEW_H