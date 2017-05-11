// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#include "GraphicsScene.h"
#include <QGraphicsView>
#include <QGraphicsItem>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsDropShadowEffect>
#include <QTimer>
#include <QApplication>
#include <QSettings>

GraphicsScene::GraphicsScene() : isGradientBackground(false), popup(nullptr)
{
    QSettings s;

    /// Background:
    // Colors
    auto lightBackColor = s.value("lightBackColor").value<QColor>();
    auto darkBackColor = s.value("darkBackColor").value<QColor>();

    // Gradient background
    if( isGradientBackground )
    {
        auto views = this->views();
        QLinearGradient gradient(0,0,0, views.size() ? views.front()->height() : 720);
        gradient.setColorAt(0.0, lightBackColor);
        gradient.setColorAt(1.0, darkBackColor);
        setBackgroundBrush(QBrush(gradient));
    }
    else
    {
        setBackgroundBrush(QBrush(darkBackColor));
    }
}

void GraphicsScene::drawBackground(QPainter *painter, const QRectF &rect)
{
    QGraphicsScene::drawBackground(painter,rect);
}

void GraphicsScene::drawForeground(QPainter *painter, const QRectF &rect)
{
    QGraphicsScene::drawForeground(painter,rect);
}

void GraphicsScene::wheelEvent(QGraphicsSceneWheelEvent *event)
{
	QGraphicsScene::wheelEvent(event);

	// block graphics view from receiving wheel events
	event->accept(); 
}

QGraphicsObject * GraphicsScene::getObjectByName(QString name)
{
    for(auto i: items()){
        auto obj = i->toGraphicsObject();
        if(!obj) continue;
        if(obj->objectName() == name){
            return obj;
        }
    }

    return nullptr;
}

void GraphicsScene::displayMessage(QString message, int time)
{
    auto textItem = addText(message, QFont("SansSerif", 24));
    textItem->setDefaultTextColor(Qt::white);

    QGraphicsDropShadowEffect * shadow = new QGraphicsDropShadowEffect();
    shadow->setColor(QColor(0,0,0,128));
    shadow->setOffset(3);
    shadow->setBlurRadius(4);
    textItem->setGraphicsEffect(shadow);
    auto textRect = textItem->sceneBoundingRect();
    textItem->moveBy(this->views().front()->width() * 0.5 - textRect.width() * 0.5,
                     this->views().front()->height() * 0.5 - textRect.height() * 0.5);
    textRect = textItem->sceneBoundingRect();
    textRect.adjust(-20,-20,20,20);

    QPainterPath textRectPath;
    textRectPath.addRoundedRect(textRect, 15, 15);
    auto rectItem = addPath(textRectPath, QPen(Qt::transparent), QBrush(QColor(0,0,0,200)));

    textItem->setZValue(500);
    rectItem->setZValue(499);

    QTimer::singleShot(time, [=]{
        removeItem(textItem);
        removeItem(rectItem);
        update();
    });
}

void GraphicsScene::showPopup(QString message)
{
    hidePopup();

    popup = addText(message, QFont("SansSerif", 24));
    auto textItem = popup;
    textItem->setDefaultTextColor(Qt::white);

    QGraphicsDropShadowEffect * shadow = new QGraphicsDropShadowEffect();
    shadow->setColor(QColor(0,0,0,128));
    shadow->setOffset(3);
    shadow->setBlurRadius(4);
    textItem->setGraphicsEffect(shadow);
    auto textRect = textItem->sceneBoundingRect();
    textItem->moveBy(this->views().front()->width() * 0.5 - textRect.width() * 0.5,
                     this->views().front()->height() * 0.5 - textRect.height() * 0.5);
    textRect = textItem->sceneBoundingRect();
    textRect.moveTopLeft(QPointF(0,0));
    textRect.adjust(-20,-20,20,20);

    QPainterPath textRectPath;
    textRectPath.addRoundedRect(textRect, 15, 15);
    auto rectItem = addPath(textRectPath, QPen(Qt::transparent), QBrush(QColor(0,0,0,200)));

    textItem->setZValue(500);
    rectItem->setZValue(499);

    rectItem->setParentItem(textItem);
    rectItem->setFlag(QGraphicsItem::ItemNegativeZStacksBehindParent);
    rectItem->setZValue(-1);

    auto tr = textItem->sceneBoundingRect();
    auto delta = textRect.center() - QRectF(0,0,tr.width(),tr.height()).center();
    rectItem->moveBy(delta.x(), delta.y());

    update();
    QApplication::processEvents();
}

void GraphicsScene::hidePopup()
{
    if(popup) {this->removeItem(popup); popup = nullptr;}

    update();
    QApplication::processEvents();
}

