#include "AppWidget.h"
#include <QtWidgets/QApplication>
#include <QStyleFactory>
#include <QTextCodec>
#include <QDebug>
#include <iostream>
#ifdef _WIN32
#include <windows.h>
#endif // _WIN32

int main(int argc, char *argv[])
{
     /*AllocConsole();
     freopen("CONOUT$", "w", stdout);
     freopen("CONOUT$", "w", stderr);*/

    QApplication a(argc, argv);
    a.setStyle(QStyleFactory::create("Fusion"));

    AppWidget w;
    w.show();
    return a.exec();
}