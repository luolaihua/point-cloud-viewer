#include "pclvisualizer.h"

#include <QDebug>

#include "./ui_pclvisualizer.h"
PCLVisualizer::PCLVisualizer(QWidget* parent)
  : QMainWindow(parent)
  , ui(new Ui::PCLVisualizer)
  , point_size(1)
{
  ui->setupUi(this);
  QString str = "PointCloudViewer";
  this->setWindowTitle(str);

  initPointCloud();
  //设置QVTK窗口
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
PCLVisualizer::test()
{
  qDebug() << "Hello World!";
}

void
PCLVisualizer::initPointCloud()
{
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
  //创建一个临时的指针保存加载的点云数据
  PointCloudT::Ptr cloud_tmp(new PointCloudT);
  //如果为空，直接返回
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
  //如果没有无效的点（如NAN，无效或非法的点），返回true
  if (cloud_tmp->is_dense)
    pcl::copyPointCloud(*cloud_tmp, *cloud_);
  else {
    PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
    std::vector<int> vec;
    pcl::removeNaNFromPointCloud(*cloud_tmp, *cloud_, vec);
  }
  qDebug() << "The number of points :" << cloud_->points.size();

  //由于加载的点云不一定是带颜色的，透明的默认为0，所以要将透明的改成最大，255
  for (PointCloudT::iterator cloud_it = cloud_->begin();
       cloud_it != cloud_->end();
       ++cloud_it) {
    //        qDebug() << cloud_it->_PointXYZRGBA::r << " " <<
    //        cloud_it->_PointXYZRGBA::g << " " << cloud_it->_PointXYZRGBA::b
    //                 << " " << cloud_it->_PointXYZRGBA::a;
    cloud_it->_PointXYZRGBA::a = 255;
  }

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
  // 找到被选中的坐标轴上的坐标的最大值和最小值
  double min, max;
  // 先设置初始值
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
