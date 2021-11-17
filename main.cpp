#include "pclvisualizer.h"

#include <QApplication>
#include <QTextCodec>
#include <vtkOutputWindow.h>
int
main(int argc, char* argv[])
{
  //    QTextCodec::setCodecForCStrings(QTextCodec::codecForName("GB2312"));
  vtkOutputWindow::SetGlobalWarningDisplay(0); //²»µ¯³övtkOutputWindow´°¿Ú
  QApplication a(argc, argv);
  PCLVisualizer w;
  w.show();
  return a.exec();
}
