#include "quickdebugmainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QuickDebugMainWindow w;
    w.show();
    return a.exec();
}
