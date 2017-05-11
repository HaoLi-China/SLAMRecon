// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#ifndef _MAINWINDOW_H
#define _MAINWINDOW_H

#include <QMainWindow>
#include <QButtonGroup>

class SlamReconManager;

namespace Ui {
class MainWindow;
}

#include <QString>

class OptionPanel;
class Tool;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void showTool(QString className);

private:
    Ui::MainWindow *ui;
    QVector<Tool*> tools;
	QButtonGroup *device_qbg;
	QButtonGroup *method_qbg;

protected:
    OptionPanel *optionPanel;

	SlamReconManager *slamReconManager;
};

#endif //_MAINWINDOW_H
