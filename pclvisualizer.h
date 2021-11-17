#ifndef PCLVISUALIZER_H
#define PCLVISUALIZER_H

#include <QAction>
#include <QComboBox>
#include <QFileDialog>
#include <QFontComboBox>
#include <QImage>
#include <QLabel>
#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QSpinBox>
#include <QTextCharFormat>
#include <QToolBar>
#include <QToolButton>

// Point Cloud Library
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <QColorDialog>
// Boost
#include <boost/math/special_functions/round.hpp>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

#include "inputdialog.h"
//使用的点云格式
typedef pcl::PointXYZRGBA PointT;
// typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

QT_BEGIN_NAMESPACE
namespace Ui {
class PCLVisualizer;
}
QT_END_NAMESPACE

class PCLVisualizer : public QMainWindow
{
  Q_OBJECT

public:
  PCLVisualizer(QWidget* parent = nullptr);
  ~PCLVisualizer();

  void createActions();  //创建动作
  void createMenus();    //创建菜单
  void createToolBars(); //创建工具栏

  //点云坐标极值
  PointT p_min, p_max;

  double maxLen;

  double getMinValue(PointT p1, PointT p2);
  double getMaxValue(PointT p1, PointT p2);

public slots:
  void test();
  //初始化数据
  void initPointCloud();
  //连接信号槽
  void connectSS();
  //保存文件
  void savePCDFile();
  void loadPCDFile();

  //选择需要控制的坐标轴
  void chooseAxis();
  //选择颜色模式
  void chooseColorMode();
  //增大 Point Size
  void IncPointSize();
  //减小 Point Size
  void DecPointSize();
  //添加/取消坐标轴
  void AddCoordinateSystem();

protected:
  int point_size;
  QColor point_color;
  //创建一个共享的PCLVisualizer 对象用于显示
  pcl::visualization::PCLVisualizer::Ptr viewer_;
  //创建一个共享指针用于保存点云
  PointCloudT::Ptr cloud_;
  /** @brief 坐标轴：0 = x | 1 = y | 2 = z */
  int filtering_axis_;

  /** @brief 颜色模式：Holds the color mode for @ref colorCloudDistances */
  int color_mode_;

  /** @brief Color point cloud on X,Y or Z axis using a Look-Up Table (LUT)
   * Computes a LUT and color the cloud accordingly, available color palettes
   * are :
   *
   *  Values are on a scale from 0 to 255:
   *  0. Blue (= 0) -> Red (= 255), this is the default value
   *  1. Green (= 0) -> Magenta (= 255)
   *  2. White (= 0) -> Red (= 255)
   *  3. Grey (< 128) / Red (> 128)
   *  4. Blue -> Green -> Red (~ rainbow)
   *
   * @warning If there's an outlier in the data the color may seem uniform
   * because of this outlier!
   * @note A boost rounding exception error will be thrown if used with a non
   * dense point cloud
   */
  void colorCloudDistances();

private slots:
  void on_actionOpen_triggered();

  void on_actionUp_triggered();

  void on_actionBottom_triggered();

  void on_actionFront_triggered();

  void on_actionBack_triggered();

  void on_actionLeft_triggered();

  void on_actionRight_triggered();

  void on_actionInput_triggered();

private:
  Ui::PCLVisualizer* ui;
  inputDialog* inputDlg;

  QMenu* fileMenu; //各项菜单栏
  QMenu* zoomMenu;
  QMenu* rotateMenu;
  QMenu* mirrorMenu;
  QImage img;
  QString fileName;
  QAction* openFileAction; //文件菜单项
  QAction* NewFileAction;
  QAction* PrintTextAction;
  QAction* PrintImageAction;
  QAction* exitAction;
  QAction* copyAction; //编辑菜单项
  QAction* cutAction;
  QAction* pasteAction;
  QAction* aboutAction;
  QAction* zoomInAction;
  QAction* zoomOutAction;
  QAction* rotate90Action; //旋转菜单项
  QAction* rotate180Action;
  QAction* rotate270Action;
  QAction* mirrorVerticalAction; //镜像菜单项
  QAction* mirrorHorizontalAction;
  QAction* undoAction;
  QAction* redoAction;
  QToolBar* fileTool; //工具栏
  QToolBar* zoomTool;
  QToolBar* rotateTool;
  QToolBar* mirrorTool;
  QToolBar* doToolBar;
  QLabel* fontLabel1; //字体设置项
  QFontComboBox* fontComboBox;
  QLabel* fontLabel2;
  QComboBox* sizeComboBox;
  QToolButton* boldBtn;
  QToolButton* italicBtn;
  QToolButton* underlineBtn;
  QToolButton* colorBtn;
  QToolBar* fontToolBar; //字体工具栏
  QLabel* listLabel;     //排序设置项
  QComboBox* listComboBox;
  QActionGroup* actGrp;
  QAction* leftAction;
  QAction* rightAction;
  QAction* centerAction;
  QAction* justifyAction;
  QToolBar* listToolBar; //排序工具栏
};
#endif // PCLVISUALIZER_H
