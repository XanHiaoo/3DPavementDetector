#include "MainWindow.h"
#include <vtkGenericOpenGLRenderWindow.h>
#include <QVTKOpenGLNativeWidget.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h> 
#include <librealsense2/rs.hpp>
#include <chrono>
#include <thread>   
#include <QTimer>
#include <pcl/visualization/vtk.h>
   
using pcl_ptr_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr;
pcl_ptr_xyzrgb points_to_pcl_xyzrgb(const rs2::points& points, const rs2::video_frame& color)
{
    pcl_ptr_xyzrgb cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());

    const uint8_t* color_data = reinterpret_cast<const uint8_t*>(color.get_data());
    const int color_bytes_per_pixel = color.get_bytes_per_pixel();

    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;

        int index = ptr - points.get_vertices();
        int color_index = index * color_bytes_per_pixel;
        p.r = color_data[color_index];
        p.g = color_data[color_index + 1];
        p.b = color_data[color_index + 2];

        ptr++;
    }

    return cloud;
}

using pcl_ptr_xyz = pcl::PointCloud<pcl::PointXYZ>::Ptr;
pcl_ptr_xyz points_to_pcl_xyz(const rs2::points& points)
{
    pcl_ptr_xyz cloud(new pcl::PointCloud<pcl::PointXYZ>);

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
    viewer_->setBackgroundColor(1.0, 1.5, 1.0);
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

void MainWindow::on_pushButton_triggered()
{
    /*pipe.start();
    timer_->start();*/
}

void MainWindow::onTimerTimeout() {   
    PavementDetector* detector = new PavementDetector();
    auto frames = pipe.wait_for_frames();
    auto depth = frames.get_depth_frame();
    auto color = frames.get_color_frame();
    points = pc.calculate(depth);
    auto cloud_xyzrgb = points_to_pcl_xyzrgb(points, color);
    auto cloud_xyz = points_to_pcl_xyz(points);
    viewer_->removeAllShapes();
    viewer_->removeAllPointClouds();
    
    int v1(0);
    viewer_->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer_->setBackgroundColor(1.0, 1.0, 1.0, v1);
    viewer_->addPointCloud(cloud_xyzrgb, "cloud_v1",v1);
    //viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0,1.0,0.0, "cloud");
    detector->detect(cloud_xyz);
    DetectionResult detectionResult = detector->getResult();

    int v2(0);
    viewer_->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer_->setBackgroundColor(1.0, 1.0, 1.0, v2);
    viewer_->addPointCloud(cloud_xyzrgb, "cloud_v2", v2);

    if (detectionResult.code == DETECTION_NG) {
        // Add each point cloud in bulge to the viewer with a green color
        for (size_t i = 0; i < detectionResult.bulge.size(); ++i)
        {
            std::string cloud_name = "bulge_" + std::to_string(i);
            viewer_->addPointCloud<pcl::PointXYZ>(detectionResult.bulge[i], cloud_name,v2);
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloud_name);
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, cloud_name);
            viewer_->addText3D("bulge" + std::to_string(i)+" " + std::to_string(detectionResult.bulgeArea[i]).substr(0, 5), 
                               pcl::PointXYZ(detectionResult.bulge[i]->points[0].x, detectionResult.bulge[i]->points[0].y,                
                               detectionResult.bulge[i]->points[0].z + 0.1), 0.02, 0.0, 0.0, 1.0, "text_bulge" + std::to_string(i),v2);

        }

        // Add each point cloud in hole to the viewer with a red color
        for (size_t i = 0; i < detectionResult.hole.size(); ++i)
        {
            std::string cloud_name = "hole_" + std::to_string(i);
            viewer_->addPointCloud<pcl::PointXYZ>(detectionResult.hole[i], cloud_name,v2);
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloud_name);
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, cloud_name);
            viewer_->addText3D("hole" + std::to_string(i) + " " + std::to_string(detectionResult.holeArea[i]).substr(0, 5), 
                                pcl::PointXYZ(detectionResult.hole[i]->points[0].x, detectionResult.hole[i]->points[0].y, 
                                detectionResult.hole[i]->points[0].z + 0.1), 0.02, 0.0, 0.0, 1.0, "text_hole" + std::to_string(i),v2);
        }
    }
    viewer_->spin();  
}

