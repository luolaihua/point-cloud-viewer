#ifndef PCLVISUALIZER_H
#define PCLVISUALIZER_H

#include <QFileDialog>
#include <QMainWindow>
// Point Cloud Library
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Boost
#include <boost/math/special_functions/round.hpp>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

//使用的点云格式
typedef pcl::PointXYZRGBA PointT;
// typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

QT_BEGIN_NAMESPACE
namespace Ui {
    class PCLVisualizer;
}
QT_END_NAMESPACE

class PCLVisualizer : public QMainWindow {
    Q_OBJECT

  public:
    PCLVisualizer(QWidget* parent = nullptr);
    ~PCLVisualizer();
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

  protected:
    int point_size;
    //创建一个共享的PCLVisualizer 对象用于显示
    pcl::visualization::PCLVisualizer::Ptr viewer_;
    //创建一个共享指针用于保存点云
    PointCloudT::Ptr cloud_;
    /** @brief 坐标轴：0 = x | 1 = y | 2 = z */
    int filtering_axis_;

    /** @brief 颜色模式：Holds the color mode for @ref colorCloudDistances */
    int color_mode_;

    /** @brief Color point cloud on X,Y or Z axis using a Look-Up Table (LUT)
     * Computes a LUT and color the cloud accordingly, available color palettes are :
     *
     *  Values are on a scale from 0 to 255:
     *  0. Blue (= 0) -> Red (= 255), this is the default value
     *  1. Green (= 0) -> Magenta (= 255)
     *  2. White (= 0) -> Red (= 255)
     *  3. Grey (< 128) / Red (> 128)
     *  4. Blue -> Green -> Red (~ rainbow)
     *
     * @warning If there's an outlier in the data the color may seem uniform because of this outlier!
     * @note A boost rounding exception error will be thrown if used with a non dense point cloud
     */
    void colorCloudDistances();

  private:
    Ui::PCLVisualizer* ui;
};
#endif  // PCLVISUALIZER_H
