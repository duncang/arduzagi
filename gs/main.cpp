


#include <iostream>
#include <QtGui>

#include <QMainWindow>

#include "ui_mainWindow.h"


int main (int argc, char *argv[])
{
	std::cout << "Starting ArduzagiGS" << std::endl;
	
	QApplication app(argc,argv);
	QMainWindow *mainWindow = new QMainWindow;
	Ui::MainWindow ui;
	ui.setupUi(mainWindow);
	mainWindow->show();

	return app.exec();
}

