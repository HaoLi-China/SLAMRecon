// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#include "SLAMTool.h"
#include "SLAMToolView.h"
#include "../GraphicsScene.h"
#include "../GraphicsView.h"

#include <QGraphicsProxyWidget>

#include <thread>

#include "../FusionEngine/Objects/FERGBDCalib.h"
#include "../FusionEngine/Utils/FELibSettings.h"
#include "../FusionEngine/FusionEngine.h"

using namespace cv;
using namespace std;
using namespace SLAMRecon;
using namespace FE;

SLAMTool::SLAMTool(const QRectF &bounds) : view(nullptr)
{
    connect(this, SIGNAL(boundsChanged()), SLOT(resizeViews()));

    setBounds(bounds);
    setObjectName("TrajTool");
}

void SLAMTool::init() {
	view = new SLAMToolView(this);

	resizeViews();
}

void SLAMTool::updateView(){
	view->update();
}

void SLAMTool::resizeViews()
{
    if(view) view->setRect(bounds);
}

void SLAMTool::setMGT(Map* pMap, CovisibilityGraph *pCograph, SpanningTree* pSpantree) {
	view->setMGT(pMap, pCograph, pSpantree);
}
