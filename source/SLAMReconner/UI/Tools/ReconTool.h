// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#ifndef _RECONTOOL_H
#define _RECONTOOL_H

#include "../Tool.h"
#include "FusionEngine.h"

class ReconToolView;

using namespace FE;

//tool to take charge of view for fusion
class ReconTool : public Tool
{
    Q_OBJECT
public:
	ReconTool(const QRectF &bounds);

    void init();

protected:
    ReconToolView* view;
    //Ui::ReconToolPanel* panel;
    QGraphicsProxyWidget* panelProxy;

public:
	void setFusionEngine(FusionEngine* fusionEngine);
	void setIfRendering(const bool& ifRendering);

public slots:
    void resizeViews();
	void updateView();

signals:
};

#endif //_RECONTOOL_H