// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#include "GraphicsView.h"
#include <QGraphicsProxyWidget>
#include <QSettings>
#include <QKeyEvent>

GraphicsView::GraphicsView(QWidget *parent) : QGraphicsView(parent)
{
    this->setHorizontalScrollBarPolicy ( Qt::ScrollBarAlwaysOff );
    this->setVerticalScrollBarPolicy ( Qt::ScrollBarAlwaysOff );
}

void GraphicsView::resizeEvent(QResizeEvent *event){
    emit(resized(QRectF(rect())));
    QGraphicsView::resizeEvent(event);
}

void GraphicsView::keyPressEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_B){
        QSettings settings;
        if(settings.value("theme", "dark") != "dark"){
            settings.setValue("theme", "dark");
            settings.setValue("darkBackColor", QColor(27, 30, 32));
            settings.setValue("lightBackColor", QColor(124, 143, 162));
        } else{
            int v = 50;
            settings.setValue("theme", "light");
            settings.setValue("darkBackColor", QColor(v, v, v));
            settings.setValue("lightBackColor", QColor(v, v, v));
        }
        settings.sync();
        emit(globalSettingsChanged());
    }

    QGraphicsView::keyPressEvent(event);
}
