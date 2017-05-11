#include "UI/MainWindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	MainWindow w;
	w.show();
    //w.setWindowIcon(QIcon(":/media/icon.png"));

    return a.exec();
}
 
