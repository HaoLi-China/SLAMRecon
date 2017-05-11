// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#include "MainWindow.h"
#include "ui_MainWindow.h"

#include <QSettings>
#include <QGraphicsProxyWidget>
#include <QPushButton>

#include "Viewer.h"
#include "GraphicsScene.h"

#include "UI/Tools/SLAMTool.h"
#include "UI/Tools/ReconTool.h"

#include "Manager/SlamReconManager.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

	slamReconManager = new SlamReconManager();

    // Global options
    {
        QSettings settings;
        // Background colors
        if (!settings.contains("theme")){
            settings.setValue("theme", "dark");
            settings.setValue("darkBackColor", QColor(27, 30, 32));
            settings.setValue("lightBackColor", QColor(124, 143, 162));
            settings.sync();
        }
    }

    // Setup view
    auto viewport1 = new Viewer();
	ui->graphicsView3D->setAlignment(Qt::AlignLeft | Qt::AlignTop);
	ui->graphicsView3D->setViewport(viewport1);
	ui->graphicsView3D->setCacheMode(QGraphicsView::CacheBackground);
	ui->graphicsView3D->setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform | QPainter::TextAntialiasing);
	ui->graphicsView3D->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);

	auto viewport2 = new Viewer();
	ui->graphicsView2D->setAlignment(Qt::AlignLeft | Qt::AlignTop);
	ui->graphicsView2D->setViewport(viewport2);
	ui->graphicsView2D->setCacheMode(QGraphicsView::CacheBackground);
	ui->graphicsView2D->setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform | QPainter::TextAntialiasing);
	ui->graphicsView2D->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);

    // Create scene3D
    auto scene3D = new GraphicsScene();
	ui->graphicsView3D->setScene(scene3D);
	scene3D->update();

	// Create scene2D
	auto scene2D = new GraphicsScene();
	ui->graphicsView2D->setScene(scene2D);
	scene2D->update();

    //Set button group
	device_qbg = new QButtonGroup();
	method_qbg = new QButtonGroup();
	device_qbg->addButton(ui->kinectOneButton);
	device_qbg->addButton(ui->filesButton);
	method_qbg->addButton(ui->slamReconButton);
	method_qbg->addButton(ui->kinfuButton);
	ui->kinectOneButton->setChecked(true);
	ui->slamReconButton->setChecked(true);

	ui->startButton->setEnabled(false);
	ui->stopButton->setEnabled(false);
	ui->saveMeshButton->setEnabled(false);
	ui->resetButton->setEnabled(false);

    // important: auto adjust the size of ui items
    this->show();

    // Add tools
    {
		auto toolsRect = ui->graphicsView3D->rect();

        // SLAMTool
        {
			auto slamTool = new SLAMTool(toolsRect);
			slamTool->connect(ui->graphicsView3D, SIGNAL(resized(QRectF)), SLOT(setBounds(QRectF)));
			slamTool->connect(slamReconManager, SIGNAL(updateSLAMView()), SLOT(updateView()));

            slamTool->setVisible(true);
			scene3D->addItem(slamTool);
            slamTool->init();

            tools.push_back(slamTool);
        }

        // ReconTool
        {
            auto reconTool = new ReconTool(toolsRect);
			reconTool->connect(ui->graphicsView2D, SIGNAL(resized(QRectF)), SLOT(setBounds(QRectF)));
			reconTool->connect(slamReconManager, SIGNAL(updateFusionView()), SLOT(updateView()));

            reconTool->setVisible(true);
			scene2D->addItem(reconTool);
            reconTool->init();

            tools.push_back(reconTool);
        }

		/*
		Button rules:
		1. startButton, stopButton, saveMeshButton, resetButton are disabled before clicking systemInitButton;
		2. After clicking systemInitButton and initializing system successfully, set systemInitButton disabled, set startButton and resetButton enabled;
		3. After clicking startButton and starting system successfully, set startButton disabled, set saveMeshButton, stopButton enabled;
		4. After clicking stopButton, set startButton enable, set stopButton disabled;
		5. After clicking saveMeshButton, set startButton enable, set stopButton disabled;
		6. After clicking resetButton, set startButton, stopButton, saveMeshButton, resetButton disabled, set systemInitButton enabled.
		*/

        // Connect tools
        {
			connect(ui->kinectOneButton, &QPushButton::pressed, [&](){
				slamReconManager->chooseKinectOne();
			});

			connect(ui->filesButton, &QPushButton::pressed, [&](){
				slamReconManager->chooseFiles();
			});

			connect(ui->slamReconButton, &QPushButton::pressed, [&](){
				slamReconManager->chooseSLAMRecon();
			});

			connect(ui->kinfuButton, &QPushButton::pressed, [&](){
				slamReconManager->chooseKinectFusion();
			});

			connect(ui->systemInitButton, &QPushButton::pressed, [&](){
				//avoid to access illegal memory
				(dynamic_cast<ReconTool*>(tools[1]))->setIfRendering(false);
				(dynamic_cast<ReconTool*>(tools[1]))->setFusionEngine(NULL);
				(dynamic_cast<SLAMTool*>(tools[0]))->setMGT(NULL, NULL, NULL);

				bool flag = slamReconManager->initSystem();

				if (flag){
					ui->systemInitButton->setEnabled(false);

					FusionEngine *fusionEngine = slamReconManager->srkPtr->fusionCompoPtr->fusionEngine;
					(dynamic_cast<ReconTool*>(tools[1]))->setFusionEngine(fusionEngine);

					if (slamReconManager->getMethodFlag() == KitsInfo::Method::SLAMRECON){
						SLAM *slamEngine = slamReconManager->srkPtr->slamCompoPtr->slamEngine;
						Map* m_pMap = slamReconManager->srkPtr->slamCompoPtr->m_pMap;
						SpanningTree* m_pSpanTree = slamReconManager->srkPtr->slamCompoPtr->m_pSpanTree;
						CovisibilityGraph* m_pCoGraph = slamReconManager->srkPtr->slamCompoPtr->m_pCoGraph;
						(dynamic_cast<SLAMTool*>(tools[0]))->setMGT(m_pMap, m_pCoGraph, m_pSpanTree);
					}

					ui->startButton->setEnabled(true);
					ui->resetButton->setEnabled(true);
				}
			});

			connect(ui->startButton, &QPushButton::pressed, [&](){
				if (slamReconManager->startSystem()){
					(dynamic_cast<ReconTool*>(tools[1]))->setIfRendering(true);
					ui->startButton->setEnabled(false);
					ui->stopButton->setEnabled(true);
					ui->saveMeshButton->setEnabled(true);
				}
			});

			connect(ui->stopButton, &QPushButton::pressed, [&](){
				slamReconManager->stopSystem();
				ui->stopButton->setEnabled(false);
				ui->startButton->setEnabled(true);
			});

			connect(ui->saveMeshButton, &QPushButton::pressed, [&](){
				slamReconManager->saveMesh();
				ui->startButton->setEnabled(true);
				ui->stopButton->setEnabled(false);
			});

			connect(ui->resetButton, &QPushButton::pressed, [&](){
				(dynamic_cast<ReconTool*>(tools[1]))->setIfRendering(false);
				(dynamic_cast<ReconTool*>(tools[1]))->setFusionEngine(NULL);
				(dynamic_cast<SLAMTool*>(tools[0]))->setMGT(NULL, NULL, NULL);

				slamReconManager->resetSystem();

				ui->startButton->setEnabled(false);
				ui->stopButton->setEnabled(false);
				ui->saveMeshButton->setEnabled(false);
				ui->resetButton->setEnabled(false);
				ui->systemInitButton->setEnabled(true);
			});
        }
    }

	scene3D->displayMessage("Welcome to SLAMRecon", 1000);

    QApplication::processEvents();
}

MainWindow::~MainWindow()
{
    delete ui;
    for (Tool* t : tools)
    {
        if (t) delete t;
    }

	if (slamReconManager != NULL){
		delete slamReconManager;
	}
}
