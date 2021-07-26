#include "debug_ui_mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    DebugUIMainWindow w;
    w.show();

    return a.exec();
}
