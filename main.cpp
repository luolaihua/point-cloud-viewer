#include "pclvisualizer.h"

#include <QApplication>
#include <vtkOutputWindow.h>
int main(int argc, char *argv[])
{
	vtkOutputWindow::SetGlobalWarningDisplay(0);//������vtkOutputWindow����
    QApplication a(argc, argv);
    PCLVisualizer w;
    w.show();
    return a.exec();
}
