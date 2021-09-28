#include "pclvisualizer.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    PCLVisualizer w;
    w.show();
    return a.exec();
}
