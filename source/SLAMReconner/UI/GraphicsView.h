// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#ifndef _GRAPHICSVIEW_H
#define _GRAPHICSVIEW_H

#include <QGraphicsView>

class GraphicsView : public QGraphicsView
{
    Q_OBJECT
public:
    GraphicsView(QWidget *parent);

protected:
	void resizeEvent(QResizeEvent *event) override;
    void keyPressEvent(QKeyEvent* event) override;

signals:
    void resized(QRectF);
    void globalSettingsChanged();
};

#endif //_GRAPHICVIEW_H