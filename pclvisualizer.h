#ifndef PCLVISUALIZER_H
#define PCLVISUALIZER_H

#include <QAction>
#include <QCloseEvent>
#include <QColor>
#include <QComboBox>
#include <QDateTime>
#include <QDebug>
#include <QDesktopWidget>
#include <QFileDialog>
#include <QFontComboBox>
#include <QImage>
#include <QLabel>
#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QProgressDialog>
#include <QRect>
#include <QSettings>
#include <QSpinBox>
#include <QString>
#include <QStringList>
#include <QTextCharFormat>
#include <QTime>
#include <QToolBar>
#include <QToolButton>
// Point Cloud Library
#include <QColorDialog>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/console/time.h> // TicToc
#include <pcl/console/time.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/random.hpp> //随机数
#include <boost/thread/thread.hpp>
#include <iostream>
#include <pcl/console/time.h> // TicToc
#include <pcl/features/normal_3d.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/flann.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_fpcs.h>  // 4PCS算法
#include <pcl/registration/ia_kfpcs.h> //K4PCS算法头文件
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pcl/search/flann_search.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/gp3.h> //贪婪投影三角化算法类定义的头文件
#include <pcl/surface/marching_cubes_hoppe.h> //移动立方体
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/mls.h>     //MLS
#include <pcl/surface/poisson.h> //泊松重建
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <vector>
#include <vtkOutputWindow.h>

#include <pcl/console/time.h>                   // TicToc
#include <pcl/filters/approximate_voxel_grid.h> // 体素滤波
#include <pcl/filters/voxel_grid.h>             // 体素滤波
#include <vtkOutputWindow.h>

#include <vtkMassProperties.h>
#include <vtkPLYReader.h>
#include <vtkSmartPointer.h>
#include <vtkTriangleFilter.h>
//可视化相关头文件
#include <vtkActor.h>
#include <vtkAutoInit.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
// Boost
#include <boost/math/special_functions/round.hpp>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

#include "inputdialog.h"
//使用的点云格式
typedef pcl::PointXYZRGBA PointTRGBA;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudTRGBA;

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

  // 打开进度窗口
  void openProgressDlg(int num);

  // 更新点云信息
  void updateCloudInfo();

  /************************************************************************/
  /* 点云预处理算法 */
  /************************************************************************/
  //全局变量 PCD文件IO
  pcl::PCDWriter writer;
  pcl::PCDReader reader;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_noise;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_guass;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_guass_down;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_out;

  //键盘事件
  int nextShow = 0;
  bool next_iteration = false;
  void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
                             void* nothing)
  {
    if (event.getKeySym() == "space" && event.keyDown()) {
      ++nextShow;
      next_iteration = true;
    }
  }

  /*
                                  下采样
  */
  void downSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
                    double leafSize = 0.01f)
  {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(leafSize, leafSize, leafSize);
    vg.filter(*cloud_filtered);
    int downsampled_points_total_size = cloud_filtered->points.size();
    // show_point_cloud(cloud_filtered, "down sampled point cloud");
    std::cout << "PointCloud after filtering has: "
              << downsampled_points_total_size << " data points." << std::endl;
  }

  /*
                                  添加高斯噪声
  */
  void GenerateGaussNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr& noise_cloud,
                          double miu = 0,
                          double sigma = 0.0005)
  {
    noise_cloud->points.resize(
      cloud->points.size()); //将点云cloud的size赋值给噪声
    noise_cloud->header = cloud->header;
    noise_cloud->width = cloud->width;
    noise_cloud->height = cloud->height;
    boost::mt19937 my_seed; // 随机数生成
    my_seed.seed(static_cast<unsigned int>(time(0)));
    boost::normal_distribution<> nd(
      miu, sigma); // 创建一个有特定期望值和标准差的正态分布：
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<>> ok(
      my_seed, nd);
    for (size_t i = 0; i < cloud->size(); ++i) {
      noise_cloud->points[i].x = cloud->points[i].x + static_cast<float>(ok());
      noise_cloud->points[i].y = cloud->points[i].y + static_cast<float>(ok());
      noise_cloud->points[i].z = cloud->points[i].z + static_cast<float>(ok());
    }
  }

  /*
                                  高斯滤波
  */

  void gaussian_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
                       double radius)
  {
    /*double radius = 0.02;*/
    // Set up the Gaussian Kernel
    pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>::Ptr kernel(
      new pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>);
    (*kernel).setSigma(4);
    (*kernel).setThresholdRelativeToSigma(4);
    std::cout << "Kernel made" << std::endl;

    // Set up the KDTree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(
      new pcl::search::KdTree<pcl::PointXYZ>);
    (*kdtree).setInputCloud(cloud);
    std::cout << "KdTree made" << std::endl;

    // Set up the Convolution Filter
    pcl::filters::Convolution3D<
      pcl::PointXYZ,
      pcl::PointXYZ,
      pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>>
      convolution;
    convolution.setKernel(*kernel);
    convolution.setInputCloud(cloud);
    convolution.setSearchMethod(kdtree);
    convolution.setRadiusSearch(radius);
    convolution.setNumberOfThreads(
      10); // important! Set Thread number for openMP
    std::cout << "Convolution Start" << std::endl;
    convolution.convolve(*cloud_filtered);
    std::cout << "Convoluted" << std::endl;
  }
  /*
                                  双边滤波器1
  */
  void bilateral_fitlter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered)
  {
    // pcl::KdTreeFLANN<PointT>::Ptr kdtree(new pcl::KdTreeFLANN<PointT>);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(
      new pcl::search::KdTree<pcl::PointXYZI>);
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new
    // pcl::search::KdTree<pcl::PointXYZ>);
    pcl::BilateralFilter<pcl::PointXYZI> bf;
    bf.setInputCloud(cloud);
    bf.setSearchMethod(kdtree);
    bf.setHalfSize(1);
    bf.setStdDev(0.01);
    bf.filter(*cloud_filtered);
  }
  void bilateralFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr output,
                       float sigma_s,
                       float sigma_r)
  {

    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree(
      new pcl::KdTreeFLANN<pcl::PointXYZ>);

    // Apply the filter
    pcl::FastBilateralFilter<pcl::PointXYZ> fbf;
    fbf.setInputCloud(input);
    fbf.setSigmaS(sigma_s);
    fbf.setSigmaR(sigma_r);
    fbf.filter(*output);
  }
  /*
                                  半径滤波器:
                                  基于密度的杂散点移除
  */
  void radius_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_out)
  {
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setRadiusSearch(0.015); //设置半径为0.04m的范围内找临近点
    sor.setMinNeighborsInRadius(35); //设置查询点的邻域点集数小于2的删除
    sor.filter(*cloud_filtered); //

    //以获取离群点数据(也就是原本滤除掉的点)
    sor.setNegative(true);

    sor.filter(*cloud_filtered_out);
  }
  /*
                                  统计滤波器:
                                  基于统计的杂散点移除
  */
  void statistical_filter(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_out)
  {
    // 创建滤波器对象
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud); //设置待滤波的点云
    sor.setMeanK(50); //设置在进行统计时考虑查询点邻居点数，K个最近邻点
    sor.setStddevMulThresh(1.0); //设置判断是否为离群点的阈值
    sor.filter(*cloud_filtered); //将滤波结果保存在cloud_filtered中

    //以获取离群点数据(也就是原本滤除掉的点)
    sor.setNegative(true);

    sor.filter(*cloud_filtered_out);
    // writer.write<pcl::PointXYZ>("table_scene_lms400_outliers.pcd",
    // *cloud_filtered, false);
  }

  void best_filter();

  /************************************************************************/
  /* 点云配准算法 */
  /************************************************************************/

  // 接下来使用的点云指针
  PointCloudT::Ptr cloud_in; // Original point cloud
  PointCloudT::Ptr cloud_tr; // Transformed point cloud
  PointCloudT::Ptr cloud_RE; // ICP output point cloud

  //使用配准基类Registration来实现多态
  pcl::Registration<PointT, PointT>::Ptr ALIGN;
  pcl::IterativeClosestPoint<PointT, PointT>::Ptr icp;

  //迭代次数
  int iterations = 1; // Default number of ICP iterations
  // bool next_iteration = false;

  /*
                  打印4*4的矩阵
  */
  void print4x4Matrix(const Eigen::Matrix4d& matrix)
  {
    printf("\nRotate Matrix :\n");
    printf(
      "    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf(
      "R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
    printf(
      "    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
    printf("\nVector :\n");
    printf("t = < %6.3f, %6.3f, %6.3f >\n\n",
           matrix(0, 3),
           matrix(1, 3),
           matrix(2, 3));
  }

  // TODO
  /*
                4*4的矩阵
*/
  QString get4x4MatrixStr(const Eigen::Matrix4d& matrix)
  {
    QString res = QString::number(matrix(0, 0), 'g', 4) + "\t" +
                  QString::number(matrix(0, 1), 'g', 4) + "\t" +
                  QString::number(matrix(0, 2), 'g', 4) + "\t" +
                  QString::number(matrix(0, 3), 'g', 4) + "\n";
    res += QString::number(matrix(1, 0), 'g', 4) + "\t" +
           QString::number(matrix(1, 1), 'g', 4) + "\t" +
           QString::number(matrix(1, 2), 'g', 4) + "\t" +
           QString::number(matrix(1, 3), 'g', 4) + "\n";
    res += QString::number(matrix(2, 0), 'g', 4) + "\t" +
           QString::number(matrix(2, 1), 'g', 4) + "\t" +
           QString::number(matrix(2, 2), 'g', 4) + "\t" +
           QString::number(matrix(2, 3), 'g', 4) + "\n";
    res += QString::number(0, 'g', 4) + "\t" + QString::number(0, 'g', 4) +
           "\t" + QString::number(0, 'g', 4) + "\t" +
           QString::number(1, 'g', 4) + "\n";
    return res;
  }
  bool isCloud2;
  /*
                  将点云旋转平移
  */
  void cloudTransform(PointCloudT::Ptr cloud_in,
                      PointCloudT::Ptr cloud_tr,
                      double theta = M_PI / 4,
                      double z = 0.1);

  /*
                迭代最近点算法
                配置ICP的参数,并设置只进行一次迭代，然后计算一次
*/
  void ICP_aligin(pcl::IterativeClosestPoint<PointT, PointT>::Ptr icp,
                  PointCloudT::Ptr cloud_in,
                  PointCloudT::Ptr cloud_RE);

  void best_aligin();

  /************************************************************************/
  /* 表面重建 */
  /************************************************************************/

  /*
        下采样点云精简

*/
  void Voxel_downsampling(PointCloudT::Ptr cloud_tr,
                          PointCloudT::Ptr filtered_cloud,
                          double leafsize = 0.001f)
  {
    // cout << "原始点云数量：" << cloud_tr->points.size() << endl;
    if (leafsize > 0) {
      pcl::VoxelGrid<pcl::PointXYZ> sor;
      sor.setInputCloud(cloud_tr);
      // double leafsize = 0.001f;//设置滤波时创建的体素体积为0.1 cm3的立方体
      sor.setLeafSize(leafsize, leafsize, leafsize); // 0.1表示精简点云到10%
      sor.filter(*filtered_cloud);
      // cout << "滤波后点云数量：" << filtered_cloud->points.size() << endl;
    } else {
      *filtered_cloud = *cloud_tr;
    }
  }
  void MLS(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
           pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals)
  {
    // 创建一个KD树
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
    // 输出文件中有PointNormal类型，用来存储移动最小二乘法算出的法线
    // pcl::PointCloud<pcl::PointNormal> mls_points;
    // 定义对象 (第二种定义类型是为了存储法线, 即使用不到也需要定义出来)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals(true);
    //设置参数
    mls.setInputCloud(cloud);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.005);
    // 曲面重建
    mls.process(*cloud_with_normals);
  }
  void Pos(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
           pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals,
           pcl::PolygonMesh& mesh)
  {
    //------------泊松重建-----------------------------
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(
      new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);
    pcl::Poisson<pcl::PointNormal> pn;
    pn.setConfidence(
      false); //是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化。
    pn.setDegree(2); //设置参数degree[1,5],值越大越精细，耗时越久。
    pn.setDepth(8);
    //树的最大深度，求解2^d x 2^d x 2^d立方体元。
    // 由于八叉树自适应采样密度，指定值仅为最大深度。

    pn.setIsoDivide(8); //用于提取ISO等值面的算法的深度
    pn.setManifold(false); //是否添加多边形的重心，当多边形三角化时。
    // 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
    pn.setOutputPolygons(
      false); //是否输出多边形网格（而不是三角化移动立方体的结果）
    pn.setSamplesPerNode(
      3.0); //设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑
    pn.setScale(1); //设置用于重构的立方体直径和样本边界立方体直径的比率。
    pn.setSolverDivide(8); //设置求解线性方程组的Gauss-Seidel迭代方法的深度
    // pn.setIndices();

    //设置搜索方法和输入点云
    pn.setSearchMethod(tree2);
    pn.setInputCloud(cloud_with_normals);

    // pn.setDegree(2); //设置参数degree[1,5],值越大越精细，耗时越久。
    // pn.setDepth(6);//设置将用于表面重建的树的最大深度
    // pn.setMinDepth(2);
    // pn.setScale(1.25);//设置用于重建的立方体的直径与样本的边界立方体直径的比值
    // pn.setSolverDivide(3);//设置块高斯-塞德尔求解器用于求解拉普拉斯方程的深度。
    // pn.setIsoDivide(8);//设置块等表面提取器用于提取等表面的深度
    // pn.setSamplesPerNode(3);//设置每个八叉树节点上最少采样点数目
    // pn.setConfidence(false);//设置置信标志，为true时，使用法线向量长度作为置信度信息，false则需要对法线进行归一化处理
    // pn.setManifold(false);//设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
    // pn.setOutputPolygons(false);//设置是否输出为多边形(而不是三角化行进立方体的结果)。
    pn.performReconstruction(mesh);
  }
  // 贪婪三角网
  void GP(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
          pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals,
          pcl::PolygonMesh& mesh)
  {

    //------------------定义搜索树对象------------------------
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(
      new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);
    //------------------贪婪投影三角化------------------
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3; //定义三角化对象

    // gp3.setSearchRadius(0.015);//设置连接点之间的最大距离（即三角形的最大边长）
    // gp3.setMu(2.5);//设置被样本点搜索其临近点的最远距离，为了适应点云密度的变化
    // gp3.setMaximumNearestNeighbors(100);//设置样本点可搜索的邻域个数
    // gp3.setMaximumSurfaceAngle(M_PI / 4); //
    // 设置某点法线方向偏离样本点法线方向的最大角度 gp3.setMinimumAngle(M_PI /
    // 18); // 设置三角化后得到三角形内角的最小角度 gp3.setMaximumAngle(2 * M_PI
    // / 3); // 设置三角化后得到三角形内角的最大角度
    // gp3.setNormalConsistency(false);//设置该参数保证法线朝向一致

    gp3.setSearchRadius(0.04); //设置连接点之间的最大距离（即三角形的最大边长）
    gp3.setMu(2.5); //设置被样本点搜索其临近点的最远距离，为了适应点云密度的变化
    gp3.setMaximumNearestNeighbors(100); //设置样本点可搜索的邻域个数
    gp3.setMaximumSurfaceAngle(
      M_PI / 4); // 设置某点法线方向偏离样本点法线方向的最大角度
    gp3.setMinimumAngle(M_PI / 18); // 设置三角化后得到三角形内角的最小角度
    gp3.setMaximumAngle(2 * M_PI / 3); // 设置三角化后得到三角形内角的最大角度
    gp3.setNormalConsistency(false); //设置该参数保证法线朝向一致

    // Get result
    gp3.setInputCloud(cloud_with_normals); //设置输入点云为有向点云
    gp3.setSearchMethod(tree2);            //设置搜索方式
    gp3.reconstruct(mesh);                 //重建提取三角化
                                           // cout << triangles;
    //------------------附加顶点信息-----------------------
    // vector<int> parts = gp3.getPartIDs();
    // vector<int> states = gp3.getPointStates();
  }

  //移动立方体
  void MC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PolygonMesh& mesh)
  {
    ////-------------------法线估计----------------------
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(5);
    n.compute(*normals);
    //----------将点云和法线放到一起------------------
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(
      new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    //---------初始化MarchingCubes对象，并设置参数-------
    pcl::MarchingCubes<pcl::PointNormal>* mc;
    mc = new pcl::MarchingCubesHoppe<pcl::PointNormal>();
    // mc = new pcl::MarchingCubesRBF<pcl::PointNormal>();
    //创建搜索树
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(
      new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);
    mc->setInputCloud(cloud_with_normals);
    //设置MarchingCubes对象的参数
    mc->setIsoLevel(0.0f); //该方法设置要提取表面的iso级别
    int resolution = 50;
    mc->setGridResolution(
      resolution, resolution, resolution); //用于设置行进立方体网格分辨率
    mc->setPercentageExtendGrid(
      0.02); //该参数定义在点云的边框和网格限制之间的网格内应该保留多少自由空间

    mc->reconstruct(mesh); //执行重构，结果保存在mesh中
  }

  void best_surface();

public slots:

  //新建工作台
  void newWorkStation();

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
  virtual void closeEvent(QCloseEvent* event);

  int point_size;
  QColor point_color;
  //创建一个共享的PCLVisualizer 对象用于显示
  pcl::visualization::PCLVisualizer::Ptr viewer_;
  //创建一个共享指针用于保存点云
  //原始点云
  PointCloudT::Ptr cloud_;
  //彩色点云
  PointCloudTRGBA::Ptr cloudRGBA_;

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

  //工具栏中的视图选择工作 6个
  void on_actionUp_triggered();

  void on_actionBottom_triggered();

  void on_actionFront_triggered();

  void on_actionBack_triggered();

  void on_actionLeft_triggered();

  void on_actionRight_triggered();

  void on_actionBGColor_triggered();

  void on_actionabout_triggered();

  void on_comboBox_Color_currentIndexChanged(const QString& arg1);

  void on_actionCoordinateSystem_triggered();

  void on_actionCameraview_triggered();

  void on_pointSizeEdt_valueChanged(int arg1);

  void on_actionbestSurface_triggered();

  void on_actionbestRemoval_triggered();

  void on_actionbestFiltering_triggered();

  void on_actionbestRegistration_triggered();

  void on_actionbestKeypoint_triggered();

  void on_actionTXT_triggered();

  void on_actionExportLog_triggered();

  void on_actionRedo_triggered();

  void on_actionquit_triggered();

  void on_actiongetAllGeo_triggered();

private:
  Ui::PCLVisualizer* ui;
  inputDialog* inputDlg;

  QString logStr;
  QStringList logList;

  QTime q_time;

  bool isRBGA;

  QColor bgColor;

  QMenu* fileMenu; //各项菜单栏
  QMenu* zoomMenu;
  QMenu* rotateMenu;
  QMenu* mirrorMenu;

  QString fileName;

  //点云文件处理动作
  QAction* addCloudAction;
  QAction* newWorkStationAction;
  QAction* exportCloud2PCDAction;
  QAction* exportCloud2PLYAction;
  QAction* newCloudAction;
  QAction* copyCloudAction;
  QAction* cutCloudAction;
  QAction* pasteCloudAction;
  QAction* searchCloudAction;
  QAction* export2CSVAction;
  QAction* export2TXTAction;
  QAction* starCloudAction;
  QAction* snapShotAction;

  QAction* outliersRemoveAction; //镜像菜单项
  QAction* filterAction;
  QAction* alignAction;
  QAction* MLSAction;
  QAction* downSampleAction;
  QAction* cloudSpliceAction;
  QAction* HistogramAction;
  QAction* surfaceAction;

  QToolBar* fileTool; //工具栏
  QToolBar* algorithmTool;
};
#endif // PCLVISUALIZER_H
