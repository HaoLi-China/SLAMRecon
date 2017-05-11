// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#ifndef _TOOL_H
#define _TOOL_H

#include <QGraphicsObject>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsProxyWidget>
#include <QPainter>

class Tool : public QGraphicsObject
{
    Q_OBJECT
    Q_PROPERTY(QRectF bounds READ boundsRect WRITE setBounds)

public:
	Tool();
	virtual ~Tool();

    QRectF boundingRect() const { return childrenBoundingRect(); }
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *);

    QRectF bounds;
    QRectF boundsRect() { return bounds; }
    bool isShowBorder;

    void init(){}

protected:

public slots:
    void setBounds(const QRectF & newBounds);
    void globalSettingsChanged(){}

signals:
    void boundsChanged();
};
#endif //_TOOL_H
