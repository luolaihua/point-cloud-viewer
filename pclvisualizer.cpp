#include "pclvisualizer.h"

#include <QDebug>

#include "./ui_pclvisualizer.h"
PCLVisualizer::PCLVisualizer(QWidget* parent) : QMainWindow(parent), ui(new Ui::PCLVisualizer) {
    ui->setupUi(this);
    cloud_.reset(new PointCloudT);
    connect(ui->pushButton_load, &QPushButton::clicked, this, &PCLVisualizer::loadPCDFile);
}

PCLVisualizer::~PCLVisualizer() { delete ui; }

void PCLVisualizer::test() { qDebug() << "Hello World!"; }

void PCLVisualizer::savePCDFile() {
    QString filename =
        QFileDialog::getSaveFileName(this, tr("Open point cloud"), "/home/", tr("Point cloud data(*.pcd *.ply)"));
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

void PCLVisualizer::loadPCDFile() {
    QString filename =
        QFileDialog::getOpenFileName(this, tr("Open point cloud"), "/home/", tr("Point cloud data (*.pcd *.ply)"));

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
    // If point cloud contains NaN values, remove them before updating the visualizer point cloud
    //    True if no points are invalid (e.g., have NaN or Inf values in any of their floating point fields).
    //如果没有无效的点（如NAN，无效或非法的点），返回true
    if (cloud_tmp->is_dense)
        pcl::copyPointCloud(*cloud_tmp, *cloud_);
    else {
        PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
        std::vector<int> vec;
        pcl::removeNaNFromPointCloud(*cloud_tmp, *cloud_, vec);
    }
    qDebug() << "The number of points :" << cloud_->points.size();
    //    colorCloudDistances ();
    //    viewer_->updatePointCloud (cloud_,"cloud");
    //    viewer_->resetCamera ();
    //    ui->qvtkWidget->update ();
}

void PCLVisualizer::chooseAxis() {}

void PCLVisualizer::chooseColorMode() {}
