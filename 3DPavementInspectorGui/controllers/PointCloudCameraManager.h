#pragma once
#include <QObject>
#include <librealsense2/rs.hpp>
#include <QThread>
#include <QString>
#include <opencv2/opencv.hpp>
#include <QStandardPaths>
#include <QDockWidget>
#include <QVTKOpenGLNativeWidget.h>
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
#include "E:/code/qt/bishe/3DPavementInspector/3DPavementDetectorDll/PavementDetector.h"

// 表示相机打开模式的枚举类型
//  Close 表示没有相机打开
enum PointCloudCameraMode {
    PointCloudcameraClose, PointCloudcameraOpen, PointCloudcameraDisConnect
};
class PointCloudCameraManager : public QObject
{
    Q_OBJECT
public:
    explicit PointCloudCameraManager(QObject* parent = nullptr);

    PointCloudCameraMode getMode() const { return mode_; }

    void setStopProcessing(bool stop_processing) { stop_processing_ = stop_processing; }
    bool getStopProcessing() const { return stop_processing_; }

    void setContext(rs2::context ctx) { ctx_ = ctx; }
    rs2::context getContext() const { return ctx_; }

    void setColorMap(rs2::colorizer color_map) { color_map_ = color_map; }
    rs2::colorizer getColorMap() const { return color_map_; }

    void setPipeline(rs2::pipeline pipe) { pipe_ = pipe; }
    rs2::pipeline getPipeline() const { return pipe_; }

    std::unique_ptr<std::thread>& getImageProcessingLoop() {
        return pointcloud_processing_loop_;
    }
    void setImageProcessingLoop(std::unique_ptr<std::thread> loop) {
        pointcloud_processing_loop_ = std::move(loop);
    }

    void getCamera();
    void openCamera();
    void closeCamera();
    void processPointCloud();

signals:
    void newPointCloud(const DetectionResult detectionResult, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb);
    void clearPlayer();

private:
    PointCloudCameraMode mode_;
    bool stop_processing_ = false;
    rs2::context ctx_;
    rs2::colorizer color_map_;
    rs2::pipeline pipe_;
    rs2::pointcloud pc_;
    rs2::points points_;
    std::unique_ptr<std::thread> pointcloud_processing_loop_;

};


