// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#include "ReconTool.h"
#include "ReconToolView.h"
#include "../../Manager/SlamReconManager.h"
#include "../GraphicsScene.h"
#include "../GraphicsView.h"
#include <QGraphicsProxyWidget>

ReconTool::ReconTool(const QRectF &bounds) : view(nullptr)
{
    connect(this, SIGNAL(boundsChanged()), SLOT(resizeViews()));

    setBounds(bounds);
    setObjectName("TrajTool");
}

void ReconTool::init()
{
	view = new ReconToolView(this);
    resizeViews();
}

void ReconTool::setFusionEngine(FusionEngine* fusionEngine)
{
	view->setFusionEngine(fusionEngine);
}

void ReconTool::setIfRendering(const bool& ifRendering)
{
	view->setIfRendering(ifRendering);
}

void ReconTool::resizeViews()
{
	if (view) {
		view->setRect(bounds);
	}
}

void ReconTool::updateView(){
	view->update();
}