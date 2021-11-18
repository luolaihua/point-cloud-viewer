#include "pclvisualizer.h"

#include <QApplication>
#include <QDateTime> //添加QDateTime头文件
#include <QMovie>
#include <QPixmap>
#include <QSplashScreen>
#include <QTextCodec>
#include <vtkOutputWindow.h>
int
main(int argc, char* argv[])
{
  vtkOutputWindow::SetGlobalWarningDisplay(0); //不弹出vtkOutputWindow窗口
  QApplication a(argc, argv);

  //启动界面
  QPixmap pixmap("1.gif");      //读取图片
  QSplashScreen splash(pixmap); //
  splash.setWindowOpacity(1);   // 设置窗口透明度

  //  QLabel label(&splash);
  //  QMovie mv("1.gif");
  //  label.setMovie(&mv);
  //  mv.start();
  //  splash.show();
  //  splash.setCursor(Qt::BlankCursor);
  //  for (int i = 0; i < 5000; i += mv.speed()) {
  //    a.processEvents(); //使程序在显示启动画面的同时仍能响应鼠标等其他事件
  //    Sleep(mv.speed());
  //  }

  //读取ini文件中上一次关闭软件时候的窗口位置和大小：
  QString wstrFilePath = qApp->applicationDirPath() + "/setting.ini";
  QSettings* settings = new QSettings(
    wstrFilePath, QSettings::IniFormat); //用QSetting获取ini文件中的数据
  int x = settings->value("WindowGeometry/x").toInt();
  int y = settings->value("WindowGeometry/y").toInt();
  int width = settings->value("WindowGeometry/width").toInt();
  int height = settings->value("WindowGeometry/height").toInt();
  QDesktopWidget* desktopWidget = QApplication::desktop();
  QRect clientRect = desktopWidget->availableGeometry();
  QRect targRect0 = QRect(clientRect.width() / 4,
                          clientRect.height() / 4,
                          clientRect.width() / 2,
                          clientRect.height() / 2);
  QRect targRect = QRect(x, y, width, height);
  if (
    width == 0 || height == 0 || x < 0 || x > clientRect.width() || y < 0 ||
    y >
      clientRect
        .height()) //如果上一次关闭软件的时候，窗口位置不正常，则本次显示在显示器的正中央
  {
    targRect = targRect0;
    qDebug() << "Position is not right:" << x << " " << y << " " << width << " "
             << height;
  }

  PCLVisualizer w;
  w.setGeometry(targRect); //设置主窗口的大小
  w.show();

  splash.finish(&w); //在主体对象初始化完成后结束启动动画

  return a.exec();
}
