
#include <QApplication>

#include "MainWindow.h"


int main(int argc, char *argv[])
{
    MainWindow   *pMainWindow;
    QApplication *pApp;
    
    pApp        = new QApplication(argc, argv);
    pMainWindow = new MainWindow;
    pMainWindow->show();

    return pApp->exec();
}


