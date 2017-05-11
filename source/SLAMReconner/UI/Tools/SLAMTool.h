// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#ifndef _SLAMTOOL_H
#define _SLAMTOOL_H

#include "../Tool.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <QLabel>

#include "../SLAMEngine/SLAM/Map.h"
#include "../SLAMEngine/SLAM/CovisibilityGraph.h"
#include "../SLAMEngine/SLAM/SpanningTree.h"

class SLAMToolView;
using namespace SLAMRecon;

//tool to take charge of view for SLAM
class SLAMTool : public Tool
{
    Q_OBJECT
public:
	SLAMTool(const QRectF &bounds);

    void init();

	void setMGT(Map* pMap, CovisibilityGraph *pCograph, SpanningTree* pSpantree);

protected:
    SLAMToolView* view;
    QGraphicsProxyWidget* panelProxy;

public slots:
    void resizeViews();
	void updateView();

signals:

};

#endif //_SLAMTOOL_H