// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#ifndef _GRAPHICSSCENE_H
#define _GRAPHICSSCENE_H

#include <QGraphicsScene>

class QGraphicsObject;

class GraphicsScene : public QGraphicsScene
{
public:
    GraphicsScene();

    QGraphicsObject *getObjectByName(QString name);

    void displayMessage(QString message, int time = 3000);

    void showPopup(QString message);
    void hidePopup();

protected:
    void drawBackground ( QPainter * painter, const QRectF & rect );
    void drawForeground ( QPainter * painter, const QRectF & rect );

    bool isGradientBackground;

    void wheelEvent(QGraphicsSceneWheelEvent *event);

    QGraphicsTextItem * popup;
};
#endif //_GRAPHICSSCENE_H
