#ifndef PCLVISUALIZER_H
#define PCLVISUALIZER_H

#include <QMainWindow>
// Point Cloud Library
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/filters/filter.h>
//#include <pcl/visualization/pcl_visualizer.h>

//// Boost
//#include <boost/math/special_functions/round.hpp>

//// Visualization Toolkit (VTK)
//#include <vtkRenderWindow.h>

QT_BEGIN_NAMESPACE
    namespace Ui { class PCLVisualizer; }
QT_END_NAMESPACE

    class PCLVisualizer : public QMainWindow
{
    Q_OBJECT

        public:
                 PCLVisualizer(QWidget *parent = nullptr);
    ~PCLVisualizer();

private:
    Ui::PCLVisualizer *ui;
};
#endif // PCLVISUALIZER_H
