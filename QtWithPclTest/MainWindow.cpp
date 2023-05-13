#include "MainWindow.h"
#include <vtkGenericOpenGLRenderWindow.h>
#include <QVTKOpenGLNativeWidget.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <pcl/io/pcd_io.h>   // 点云文件 I/O
#include <pcl/point_types.h> // PCL支持的点类型
#include <librealsense2/rs.hpp>
#include <chrono>
#include <thread>   
#include <QTimer>
#include <pcl/visualization/vtk.h>
   
using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
pcl_ptr points_to_pcl(const rs2::points& points)
{
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
    qvtkWidget = new QVTKOpenGLNativeWidget;
    vtkSmartPointer<vtkRenderer> ren = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer< vtkGenericOpenGLRenderWindow> renWin = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renWin->AddRenderer(ren);
    qvtkWidget->setRenderWindow(renWin);
    qvtkWidget->paintingActive();
    viewer_.reset(new pcl::visualization::PCLVisualizer(ren, renWin, "3D Viewer", false));
    viewer_->setBackgroundColor(1.0, 1.0, 1.0);
    viewer_->setCameraPosition(0, 0, -2, 0, 0, 0);
    viewer_->setupInteractor(qvtkWidget->interactor(), qvtkWidget->renderWindow());


    ui.verticalLayout->addWidget(qvtkWidget);

    timer_ = new QTimer(this);
    timer_->setInterval(33); // 50ms
    connect(timer_, &QTimer::timeout, this, &MainWindow::onTimerTimeout);
    pipe.start();
    timer_->start();
    
}

MainWindow::~MainWindow()
{
}
using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

void MainWindow::on_pushButton_triggered()
{
    /*pipe.start();
    timer_->start();*/
}

void MainWindow::onTimerTimeout() {      
    auto frames = pipe.wait_for_frames();
    auto depth = frames.get_depth_frame();
    points = pc.calculate(depth);
    auto cloud = points_to_pcl(points);
    viewer_->removeAllShapes();
    viewer_->removeAllPointClouds();
    viewer_->addPointCloud(cloud, "cloud");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0,1.0,0.0, "cloud");
    viewer_->spin();  
}

