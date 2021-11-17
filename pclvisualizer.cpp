#include "pclvisualizer.h"

#include "./ui_pclvisualizer.h"
#include <QColor>
#include <QColorDialog>
#include <QDebug>
#include <QFile>
#include <QFileDialog>
#include <QPainter>
#include <QTextList>
#include <QTextStream>
PCLVisualizer::PCLVisualizer(QWidget* parent)
  : QMainWindow(parent)
  , point_size(1)
  , ui(new Ui::PCLVisualizer)

{
  ui->setupUi(this);
  //  qDebug() << "666777888";
  QString str = "PointCloudViewer";
  this->setWindowTitle(str);

  //  //创建动作，工具栏以及菜单栏
  //  createActions();
  //  createMenus();
  //  createToolBars();

  initPointCloud();
  //璁剧疆QVTK绐楀彛
  viewer_.reset(new pcl::visualization::PCLVisualizer("viewer", false));
  viewer_->setBackgroundColor(255, 255, 255);
  ui->qvtkWidget->SetRenderWindow(viewer_->getRenderWindow());
  viewer_->setupInteractor(ui->qvtkWidget->GetInteractor(),
                           ui->qvtkWidget->GetRenderWindow());
  ui->qvtkWidget->update();
  connectSS();

  // Color the randomly generated cloud
  colorCloudDistances();

  viewer_->addPointCloud(cloud_, "cloud");
  viewer_->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size);
  // viewer_->addCoordinateSystem(1);
  viewer_->resetCamera();
  viewer_->setLookUpTableID("cloud");
  ui->qvtkWidget->update();
}

PCLVisualizer::~PCLVisualizer()
{
  delete ui;
}

void
PCLVisualizer::createActions()
{
  //鈥滄墦寮€鈥濆姩浣?
  openFileAction =
    new QAction(QIcon(":/images/Excel_in.png"), "测试", this); //(a)
  openFileAction->setShortcut(tr("Ctrl+O"));                   //(b)
  openFileAction->setStatusTip(tr("测试"));                    //(c)

  //鈥滄柊寤衡€濆姩浣?
  NewFileAction = new QAction(QIcon("new.png"), tr("测试"), this);
  NewFileAction->setShortcut(tr("Ctrl+N"));
  NewFileAction->setStatusTip(tr("测试"));

  //鈥滈€€鍑衡€濆姩浣?
  exitAction = new QAction(tr("测试?"), this);
  exitAction->setShortcut(tr("Ctrl+Q"));
  exitAction->setStatusTip(tr("测试"));

  //鈥滃鍒垛€濆姩浣?
  copyAction = new QAction(QIcon("copy.png"), tr("测试"), this);
  copyAction->setShortcut(tr("Ctrl+C"));
  copyAction->setStatusTip(tr("测试"));

  //鈥滃壀鍒団€濆姩浣?
  cutAction = new QAction(QIcon("cut.png"), tr("测试"), this);
  cutAction->setShortcut(tr("Ctrl+X"));
  cutAction->setStatusTip(tr("测试"));

  //鈥滅矘璐粹€濆姩浣?
  pasteAction = new QAction(QIcon("paste.png"), tr("测试"), this);
  pasteAction->setShortcut(tr("Ctrl+V"));
  pasteAction->setStatusTip(tr("测试"));

  //鈥滃叧浜庘€濆姩浣?
  aboutAction = new QAction(tr("测试"), this);

  //鈥滄墦鍗版枃鏈€濆姩浣?
  PrintTextAction = new QAction(QIcon("printText.png"), tr("测试"), this);
  PrintTextAction->setStatusTip(tr("测试"));

  //鈥滄墦鍗板浘鍍忊€濆姩浣?
  PrintImageAction = new QAction(QIcon("printImage.png"), tr("测试"), this);
  PrintImageAction->setStatusTip(tr("测试"));

  //鈥滄斁澶р€濆姩浣?
  zoomInAction = new QAction(QIcon("zoomin.png"), tr("测试"), this);
  zoomInAction->setStatusTip(tr("测试?"));

  //鈥滅缉灏忊€濆姩浣?
  zoomOutAction = new QAction(QIcon("zoomout.png"), tr("测试"), this);
  zoomOutAction->setStatusTip(tr("测试?"));

  //瀹炵幇鍥惧儚鏃嬭浆鐨勫姩浣滐紙Action锛?
  //鏃嬭浆90掳
  rotate90Action = new QAction(QIcon("rotate90.png"), tr("测试"), this);
  rotate90Action->setStatusTip(tr("测试"));

  //鏃嬭浆180掳
  rotate180Action = new QAction(QIcon("rotate180.png"), tr("测试"), this);
  rotate180Action->setStatusTip(tr("测试"));

  //鏃嬭浆270掳
  rotate270Action = new QAction(QIcon("rotate270.png"), tr("测试"), this);
  rotate270Action->setStatusTip(tr("测试"));

  //瀹炵幇鍥惧儚闀滃儚鐨勫姩浣滐紙Action锛?
  //绾靛悜闀滃儚
  mirrorVerticalAction =
    new QAction(QIcon("mirrorVertical.png"), tr("测试"), this);
  mirrorVerticalAction->setStatusTip(tr("测试?"));

  //妯悜闀滃儚
  mirrorHorizontalAction =
    new QAction(QIcon("mirrorHorizontal.png"), tr("测试"), this);
  mirrorHorizontalAction->setStatusTip(tr("测试?"));

  //鎺掑簭锛氬乏瀵归綈銆佸彸瀵归綈銆佸眳涓拰涓ょ瀵归綈
  actGrp = new QActionGroup(this);
  leftAction = new QAction(QIcon("left.png"), "测试?", actGrp);
  leftAction->setCheckable(true);
  rightAction = new QAction(QIcon("right.png"), "测试?", actGrp);
  rightAction->setCheckable(true);
  centerAction = new QAction(QIcon("center.png"), "测试", actGrp);
  centerAction->setCheckable(true);
  justifyAction = new QAction(QIcon("justify.png"), "测试", actGrp);
  justifyAction->setCheckable(true);

  //瀹炵幇鎾ら攢鍜屾仮澶嶇殑鍔ㄤ綔锛圓ction锛?
  //鎾ら攢鍜屾仮澶?
  undoAction = new QAction(QIcon("undo.png"), "测试", this);
  redoAction = new QAction(QIcon("redo.png"), "测试", this);
}

void
PCLVisualizer::createMenus()
{
  //鏂囦欢鑿滃崟
  fileMenu = menuBar()->addMenu(tr("测试")); //(a)
  fileMenu->addAction(openFileAction);       //(b)
  fileMenu->addAction(NewFileAction);
  fileMenu->addAction(PrintTextAction);
  fileMenu->addAction(PrintImageAction);
  fileMenu->addSeparator();
  fileMenu->addAction(exitAction);
  //缂╂斁鑿滃崟
  zoomMenu = menuBar()->addMenu(tr("测试"));
  zoomMenu->addAction(copyAction);
  zoomMenu->addAction(cutAction);
  zoomMenu->addAction(pasteAction);
  zoomMenu->addAction(aboutAction);
  zoomMenu->addSeparator();
  zoomMenu->addAction(zoomInAction);
  zoomMenu->addAction(zoomOutAction);
  //鏃嬭浆鑿滃崟
  rotateMenu = menuBar()->addMenu(tr("测试"));
  rotateMenu->addAction(rotate90Action);
  rotateMenu->addAction(rotate180Action);
  rotateMenu->addAction(rotate270Action);
  //闀滃儚鑿滃崟
  mirrorMenu = menuBar()->addMenu(tr("测试"));
  mirrorMenu->addAction(mirrorVerticalAction);
  mirrorMenu->addAction(mirrorHorizontalAction);
}

void
PCLVisualizer::createToolBars()
{
  //鏂囦欢宸ュ叿鏉?
  fileTool = addToolBar("File");       //(a)
  fileTool->addAction(openFileAction); //(b)
  fileTool->addAction(NewFileAction);
  fileTool->addAction(PrintTextAction);
  fileTool->addAction(PrintImageAction);
  //缂栬緫宸ュ叿鏉?
  zoomTool = addToolBar("Edit");
  zoomTool->addAction(copyAction);
  zoomTool->addAction(cutAction);
  zoomTool->addAction(pasteAction);
  zoomTool->addSeparator();
  zoomTool->addAction(zoomInAction);
  zoomTool->addAction(zoomOutAction);
  //鏃嬭浆宸ュ叿鏉?
  rotateTool = addToolBar("rotate");
  rotateTool->addAction(rotate90Action);
  rotateTool->addAction(rotate180Action);
  rotateTool->addAction(rotate270Action);
  //鎾ら攢鍜岄噸鍋氬伐鍏锋潯
  doToolBar = addToolBar("doEdit");
  doToolBar->addAction(undoAction);
  doToolBar->addAction(redoAction);
  //瀛椾綋宸ュ叿鏉?
  fontToolBar = addToolBar("Font");
  fontToolBar->addWidget(fontLabel1);
  fontToolBar->addWidget(fontComboBox);
  fontToolBar->addWidget(fontLabel2);
  fontToolBar->addWidget(sizeComboBox);
  fontToolBar->addSeparator();
  fontToolBar->addWidget(boldBtn);
  fontToolBar->addWidget(italicBtn);
  fontToolBar->addWidget(underlineBtn);
  fontToolBar->addSeparator();
  fontToolBar->addWidget(colorBtn);
  //鎺掑簭宸ュ叿鏉?
  listToolBar = addToolBar("list");
  listToolBar->addWidget(listLabel);
  listToolBar->addWidget(listComboBox);
  listToolBar->addSeparator();
  listToolBar->addActions(actGrp->actions());
}

void
PCLVisualizer::test()
{
  qDebug() << "Hello World!";
}

void
PCLVisualizer::initPointCloud()
{
  inputDlg = new inputDialog(this);
  // Setup the cloud pointer
  cloud_.reset(new PointCloudT);
  // The number of points in the cloud
  cloud_->resize(500);

  // Fill the cloud with random points
  for (size_t i = 0; i < cloud_->points.size(); ++i) {
    cloud_->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud_->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud_->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
  }

  pcl::getMinMax3D(*cloud_, p_min, p_max);
  maxLen = getMaxValue(p_max, p_min);
}

void
PCLVisualizer::connectSS()
{
  connect(ui->pushButton_inc,
          &QPushButton::clicked,
          this,
          &PCLVisualizer::IncPointSize);
  connect(ui->actionload_point_cloud,
          &QAction::triggered,
          this,
          &PCLVisualizer::loadPCDFile);
  connect(ui->actionsave_point_cloud,
          &QAction::triggered,
          this,
          &PCLVisualizer::savePCDFile);
  connect(ui->actionCoordinateSystem,
          &QAction::triggered,
          this,
          &PCLVisualizer::AddCoordinateSystem);
  // Connect "Load" and "Save" buttons and their functions
  connect(ui->pushButton_dec,
          &QPushButton::clicked,
          this,
          &PCLVisualizer::DecPointSize);

  // Connect X,Y,Z radio buttons and their functions
  connect(ui->radioButton_x,
          &QRadioButton::clicked,
          this,
          &PCLVisualizer::chooseAxis);
  connect(ui->radioButton_y,
          &QRadioButton::clicked,
          this,
          &PCLVisualizer::chooseAxis);
  connect(ui->radioButton_z,
          &QRadioButton::clicked,
          this,
          &PCLVisualizer::chooseAxis);

  connect(ui->radioButton_BlueRed,
          &QRadioButton::clicked,
          this,
          &PCLVisualizer::chooseColorMode);
  connect(ui->radioButton_GreenMagenta,
          &QRadioButton::clicked,
          this,
          &PCLVisualizer::chooseColorMode);
  connect(ui->radioButton_WhiteRed,
          &QRadioButton::clicked,
          this,
          &PCLVisualizer::chooseColorMode);
  connect(ui->radioButton_GreyRed,
          &QRadioButton::clicked,
          this,
          &PCLVisualizer::chooseColorMode);
  connect(ui->radioButton_Rainbow,
          &QRadioButton::clicked,
          this,
          &PCLVisualizer::chooseColorMode);
  connect(ui->radioButton_others,
          &QRadioButton::clicked,
          this,
          &PCLVisualizer::chooseColorMode);
}

void
PCLVisualizer::savePCDFile()
{
  QString filename =
    QFileDialog::getSaveFileName(this,
                                 tr("Open point cloud"),
                                 "/home/",
                                 tr("Point cloud data(*.pcd *.ply)"));
  PCL_INFO("File chosen: %s\n", filename.toStdString().c_str());

  if (filename.isEmpty())
    return;
  int return_status;
  if (filename.endsWith(".pcd", Qt::CaseInsensitive))
    return_status = pcl::io::savePCDFileBinary(filename.toStdString(), *cloud_);
  else if (filename.endsWith(".ply", Qt::CaseInsensitive))
    return_status = pcl::io::savePLYFileBinary(filename.toStdString(), *cloud_);
  else {
    filename.append(".ply");
    return_status = pcl::io::savePLYFileBinary(filename.toStdString(), *cloud_);
  }
  if (return_status != 0) {
    PCL_ERROR("Error writing point cloud %s\n", filename.toStdString().c_str());
    return;
  }
}

void
PCLVisualizer::loadPCDFile()
{
  QString filename =
    QFileDialog::getOpenFileName(this,
                                 tr("Open point cloud"),
                                 "/home/",
                                 tr("Point cloud data (*.pcd *.ply)"));

  PCL_WARN("File chosen: %s\n", filename.toStdString().c_str());
  //鍒涘缓涓€涓复鏃剁殑鎸囬拡淇濆瓨鍔犺浇鐨勭偣浜戞暟鎹?
  PointCloudT::Ptr cloud_tmp(new PointCloudT);
  //濡傛灉涓虹┖锛岀洿鎺ヨ繑鍥?
  if (filename.isEmpty())
    return;

  int return_status;
  if (filename.endsWith(".pcd", Qt::CaseInsensitive))
    return_status = pcl::io::loadPCDFile(filename.toStdString(), *cloud_tmp);
  else
    return_status = pcl::io::loadPLYFile(filename.toStdString(), *cloud_tmp);

  if (return_status != 0) {
    PCL_ERROR("Error reading point cloud %s\n", filename.toStdString().c_str());
    return;
  }
  PCL_WARN("file has loaded\n");
  // If point cloud contains NaN values, remove them before updating the
  // visualizer point cloud
  //    True if no points are invalid (e.g., have NaN or Inf values in any of
  //    their floating point fields).
  //濡傛灉娌℃湁鏃犳晥鐨勭偣锛堝NAN锛屾棤鏁堟垨闈炴硶鐨勭偣锛夛紝杩斿洖true
  if (cloud_tmp->is_dense)
    pcl::copyPointCloud(*cloud_tmp, *cloud_);
  else {
    PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
    std::vector<int> vec;
    pcl::removeNaNFromPointCloud(*cloud_tmp, *cloud_, vec);
  }
  qDebug() << "The number of points :" << cloud_->points.size();

  //鐢变簬鍔犺浇鐨勭偣浜戜笉涓€瀹氭槸甯﹂鑹茬殑锛岄€忔槑鐨勯粯璁や负0锛屾墍浠ヨ灏嗛€忔槑鐨勬敼鎴愭渶澶э紝255
  for (PointCloudT::iterator cloud_it = cloud_->begin();
       cloud_it != cloud_->end();
       ++cloud_it) {
    //        qDebug() << cloud_it->_PointXYZRGBA::r << " " <<
    //        cloud_it->_PointXYZRGBA::g << " " << cloud_it->_PointXYZRGBA::b
    //                 << " " << cloud_it->_PointXYZRGBA::a;
    cloud_it->_PointXYZRGBA::a = 255;
  }

  pcl::getMinMax3D(*cloud_, p_min, p_max);
  maxLen = getMaxValue(p_max, p_min);

  colorCloudDistances();
  viewer_->updatePointCloud(cloud_, "cloud");
  viewer_->resetCamera();
  ui->qvtkWidget->update();
}

void
PCLVisualizer::chooseAxis()
{
  if (color_mode_ == 5)
    return;
  // Only 1 of the button can be checked at the time (mutual exclusivity) in a
  // group of radio buttons
  if (ui->radioButton_x->isChecked()) {
    PCL_INFO("x filtering chosen\n");
    filtering_axis_ = 0;
  } else if (ui->radioButton_y->isChecked()) {
    PCL_INFO("y filtering chosen\n");
    filtering_axis_ = 1;
  } else {
    PCL_INFO("z filtering chosen\n");
    filtering_axis_ = 2;
  }

  colorCloudDistances();
  viewer_->updatePointCloud(cloud_, "cloud");
  ui->qvtkWidget->update();
}

void
PCLVisualizer::chooseColorMode()
{
  // Only 1 of the button can be checked at the time (mutual exclusivity) in a
  // group of radio buttons
  if (ui->radioButton_BlueRed->isChecked()) {
    PCL_INFO("Blue -> Red LUT chosen\n");
    color_mode_ = 0;
  } else if (ui->radioButton_GreenMagenta->isChecked()) {
    PCL_INFO("Green -> Magenta LUT chosen\n");
    color_mode_ = 1;
  } else if (ui->radioButton_WhiteRed->isChecked()) {
    PCL_INFO("White -> Red LUT chosen\n");
    color_mode_ = 2;
  } else if (ui->radioButton_GreyRed->isChecked()) {
    PCL_INFO("Grey / Red LUT chosen\n");
    color_mode_ = 3;
  } else if (ui->radioButton_Rainbow->isChecked()) {
    PCL_INFO("Rainbow LUT chosen\n");
    color_mode_ = 4;
  } else {
    PCL_INFO("Full color chosen\n");
    color_mode_ = 5;
    QColor c = QColorDialog::getColor(Qt::red);
    if (c.isValid()) {
      point_color = c;
      qDebug() << "RBG: " << c.red() << " " << c.green() << " " << c.blue();
    }
  }

  colorCloudDistances();
  viewer_->updatePointCloud(cloud_, "cloud");
  ui->qvtkWidget->update();
}

void
PCLVisualizer::IncPointSize()
{
  viewer_->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ++point_size);

  qDebug() << "point_size = " << point_size;
  ui->label_pointSize->setText(QString::number(point_size));
  // viewer_->updatePointCloud(cloud_, "cloud");
  ui->qvtkWidget->update();
}

void
PCLVisualizer::DecPointSize()
{
  if (point_size == 1)
    return;
  viewer_->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, --point_size);
  ui->label_pointSize->setText(QString::number(point_size));
  qDebug() << "point_size = " << point_size;
  // viewer_->updatePointCloud(cloud_, "cloud");
  ui->qvtkWidget->update();
}

void
PCLVisualizer::AddCoordinateSystem()
{
  QString str = ui->actionCoordinateSystem->text();
  if (str.compare("CoordinateSystem [OFF]") == 0) {
    qDebug() << str;
    ui->actionCoordinateSystem->setText("CoordinateSystem [ON]");
    viewer_->addCoordinateSystem();
  } else {
    ui->actionCoordinateSystem->setText("CoordinateSystem [OFF]");
    viewer_->removeCoordinateSystem();
  }
  ui->qvtkWidget->update();
}

void
PCLVisualizer::colorCloudDistances()
{
  // 鎵惧埌琚€変腑鐨勫潗鏍囪酱涓婄殑鍧愭爣鐨勬渶澶у€煎拰鏈€灏忓€?
  double min, max;
  // 鍏堣缃垵濮嬪€?
  switch (filtering_axis_) {
    case 0: // x
      min = cloud_->points[0].x;
      max = cloud_->points[0].x;
      break;
    case 1: // y
      min = cloud_->points[0].y;
      max = cloud_->points[0].y;
      break;
    default: // z
      min = cloud_->points[0].z;
      max = cloud_->points[0].z;
      break;
  }
  // Search for the minimum/maximum
  for (PointCloudT::iterator cloud_it = cloud_->begin();
       cloud_it != cloud_->end();
       ++cloud_it) {
    switch (filtering_axis_) {
      case 0: // x
        if (min > cloud_it->x)
          min = cloud_it->x;

        if (max < cloud_it->x)
          max = cloud_it->x;
        break;
      case 1: // y
        if (min > cloud_it->y)
          min = cloud_it->y;

        if (max < cloud_it->y)
          max = cloud_it->y;
        break;
      default: // z
        if (min > cloud_it->z)
          min = cloud_it->z;

        if (max < cloud_it->z)
          max = cloud_it->z;
        break;
    }
  }
  // Compute LUT scaling to fit the full histogram spectrum
  double lut_scale = 255.0 / (max - min); // max is 255, min is 0

  if (min ==
      max) // In case the cloud is flat on the chosen direction (x,y or z)
    lut_scale = 1.0; // Avoid rounding error in boost

  for (PointCloudT::iterator cloud_it = cloud_->begin();
       cloud_it != cloud_->end();
       ++cloud_it) {
    int value;
    switch (filtering_axis_) {
      case 0: // x
        value = boost::math::iround(
          (cloud_it->x - min) *
          lut_scale); // Round the number to the closest integer
        break;
      case 1: // y
        value = boost::math::iround((cloud_it->y - min) * lut_scale);
        break;
      default: // z
        value = boost::math::iround((cloud_it->z - min) * lut_scale);
        break;
    }

    // Apply color to the cloud
    switch (color_mode_) {
      case 0:
        // Blue (= min) -> Red (= max)
        cloud_it->r = value;
        cloud_it->g = 0;
        cloud_it->b = 255 - value;
        break;
      case 1:
        // Green (= min) -> Magenta (= max)
        cloud_it->r = value;
        cloud_it->g = 255 - value;
        cloud_it->b = value;
        break;
      case 2:
        // White (= min) -> Red (= max)
        cloud_it->r = 255;
        cloud_it->g = 255 - value;
        cloud_it->b = 255 - value;
        break;
      case 3:
        // Grey (< 128) / Red (> 128)
        if (value > 128) {
          cloud_it->r = 255;
          cloud_it->g = 0;
          cloud_it->b = 0;
        } else {
          cloud_it->r = 128;
          cloud_it->g = 128;
          cloud_it->b = 128;
        }
        break;
      case 5:
        cloud_it->r = point_color.red();
        cloud_it->g = point_color.green();
        cloud_it->b = point_color.blue();
        break;
      default:
        // Blue -> Green -> Red (~ rainbow)
        cloud_it->r =
          value > 128 ? (value - 128) * 2 : 0; // r[128] = 0, r[255] = 255
        cloud_it->g =
          value < 128
            ? 2 * value
            : 255 - ((value - 128) * 2); // g[0] = 0, g[128] = 255, g[255] = 0
        cloud_it->b =
          value < 128 ? 255 - (2 * value) : 0; // b[0] = 255, b[128] = 0
    }
  }
}

void
PCLVisualizer::on_actionOpen_triggered()
{
  loadPCDFile();
  // qDebug() << "on_actionOpen_triggered";
}

void
PCLVisualizer::on_actionUp_triggered()
{
  if (!cloud_->empty()) {
    viewer_->setCameraPosition(0.5 * (p_min.x + p_max.x),
                               0.5 * (p_min.y + p_max.y),
                               p_max.z + 2 * maxLen,
                               0.5 * (p_min.x + p_max.x),
                               0.5 * (p_min.y + p_max.y),
                               p_max.z,
                               0,
                               1,
                               0);
    ui->qvtkWidget->update();
  }
}

void
PCLVisualizer::on_actionBottom_triggered()
{
  if (!cloud_->empty()) {
    viewer_->setCameraPosition(0.5 * (p_min.x + p_max.x),
                               0.5 * (p_min.y + p_max.y),
                               p_min.z - 2 * maxLen,
                               0.5 * (p_min.x + p_max.x),
                               0.5 * (p_min.y + p_max.y),
                               p_min.z,
                               0,
                               1,
                               0);
    ui->qvtkWidget->update();
  }
}

void
PCLVisualizer::on_actionFront_triggered()
{
  if (!cloud_->empty()) {
    viewer_->setCameraPosition(0.5 * (p_min.x + p_max.x),
                               p_min.y - 2 * maxLen,
                               0.5 * (p_min.z + p_max.z),
                               0.5 * (p_min.x + p_max.x),
                               p_min.y,
                               0.5 * (p_min.z + p_max.z),
                               0,
                               0,
                               1);
    ui->qvtkWidget->update();
  }
}

void
PCLVisualizer::on_actionBack_triggered()
{
  if (!cloud_->empty()) {
    viewer_->setCameraPosition(0.5 * (p_min.x + p_max.x),
                               p_max.y + 2 * maxLen,
                               0.5 * (p_min.z + p_max.z),
                               0.5 * (p_min.x + p_max.x),
                               p_min.y,
                               0.5 * (p_min.z + p_max.z),
                               0,
                               0,
                               1);
    ui->qvtkWidget->update();
  }
}

void
PCLVisualizer::on_actionLeft_triggered()
{
  if (!cloud_->empty()) {
    viewer_->setCameraPosition(p_min.x - 2 * maxLen,
                               0.5 * (p_min.y + p_max.y),
                               0.5 * (p_min.z + p_max.z),
                               p_max.x,
                               0.5 * (p_min.y + p_max.y),
                               0.5 * (p_min.z + p_max.z),
                               0,
                               0,
                               1);
    ui->qvtkWidget->update();
  }
}

void
PCLVisualizer::on_actionRight_triggered()
{
  if (!cloud_->empty()) {
    viewer_->setCameraPosition(p_max.x + 2 * maxLen,
                               0.5 * (p_min.y + p_max.y),
                               0.5 * (p_min.z + p_max.z),
                               p_max.x,
                               0.5 * (p_min.y + p_max.y),
                               0.5 * (p_min.z + p_max.z),
                               0,
                               0,
                               1);
    ui->qvtkWidget->update();
  }
}

double
PCLVisualizer::getMinValue(PointT p1, PointT p2)
{
  double min = 0;

  if (p1.x - p2.x > p1.y - p2.y) {
    min = p1.y - p2.y;
  } else {
    min = p1.x - p2.x;
  }

  if (min > p1.z - p2.z) {
    min = p1.z - p2.z;
  }

  return min;
}

double
PCLVisualizer::getMaxValue(PointT p1, PointT p2)
{
  double max = 0;

  if (p1.x - p2.x > p1.y - p2.y) {
    max = p1.x - p2.x;

  } else {
    max = p1.y - p2.y;
  }

  if (max < p1.z - p2.z) {
    max = p1.z - p2.z;
  }

  return max;
}

void
PCLVisualizer::on_actionInput_triggered()
{
  inputDlg->show();
}

void
PCLVisualizer::on_actionRotate0_triggered()
{
  qDebug() << "on_actionRotate0_triggered";
}

void
PCLVisualizer::on_actionRotate90_triggered()
{
  qDebug() << "on_actionRotate90_triggered";
}

void
PCLVisualizer::on_actionRotate270_triggered()
{
  qDebug() << "on_actionRotate270_triggered";
}

void
PCLVisualizer::on_actionRotate180_triggered()
{
  qDebug() << "on_actionRotate180_triggered";
}

void
PCLVisualizer::on_actionzoomIn_triggered()
{
  qDebug() << "on_actionzoomIn_triggered";
}

void
PCLVisualizer::on_actionzoomOut_triggered()
{
  qDebug() << "on_actionzoomOut_triggered";
}

void
PCLVisualizer::on_actionRedo_triggered()
{
  qDebug() << "on_actionRedo_triggered";
}

void
PCLVisualizer::on_actionUndo_triggered()
{
  qDebug() << "on_actionUndo_triggered";
}

void
PCLVisualizer::on_actionResetPos_triggered()
{
  qDebug() << "on_actionResetPos_triggered";
}
