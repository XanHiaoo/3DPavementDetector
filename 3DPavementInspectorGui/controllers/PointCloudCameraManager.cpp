#include "PointCloudCameraManager.h"
#include <QtDebug>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonDocument>
#include <QMessageBox>
#include <QFile>
#include <chrono>


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
PointCloudCameraManager::PointCloudCameraManager(QObject* parent) : QObject(parent)
{
    mode_ = PointCloudcameraDisConnect;
}

void PointCloudCameraManager::getCamera() {
    auto cameraList = ctx_.query_devices(); // Get a snapshot of currently connected devices
    if (cameraList.size() == 0) {
        mode_ = PointCloudcameraDisConnect;
    }
    else {
        mode_ = PointCloudcameraClose;
    }
}

void PointCloudCameraManager::openCamera() {
    //// Declare depth colorizer for pretty visualization of depth data
    //color_map_ = rs2::colorizer();

    //// Declare RealSense pipeline, encapsulating the actual device and sensors
    //pipe_ = rs2::pipeline();
    //// Start streaming with default recommended configuration
    //pipe_.start();

    //// Connect the signal to the slot
    //if (!pointcloud_processing_loop_ || !pointcloud_processing_loop_->joinable()) {
    //    // If the image processing loop is not already running, start it
    //    stop_processing_ = false;
    //    pointcloud_processing_loop_ = std::make_unique<std::thread>(&PointCloudCameraManager::processPointCloud, this);
    //    mode_ = PointCloudcameraOpen;
    //}
    mode_ = PointCloudcameraOpen;
}

void PointCloudCameraManager::closeCamera() {
    /*stop_processing_ = true;
    if (stop_processing_ && pointcloud_processing_loop_->joinable()) {
        pointcloud_processing_loop_->join();
    }*/
    mode_ = PointCloudcameraClose;
    emit clearPlayer();
}

void PointCloudCameraManager::processPointCloud()
{
    PavementDetector* detector = new PavementDetector();
    while (!stop_processing_) {
        try { 
            auto start_time = std::chrono::steady_clock::now();
            
            auto frames = pipe_.wait_for_frames();
            auto depth = frames.get_depth_frame();
            auto color = frames.get_color_frame();
            points_ = pc_.calculate(depth);
            auto cloud_xyzrgb = points_to_pcl_xyzrgb(points_, color);
            auto cloud_xyz = points_to_pcl_xyz(points_);
            detector->detect(cloud_xyz);
            DetectionResult detectionResult = detector->getResult();
            emit newPointCloud(detectionResult, cloud_xyzrgb);
            
            auto end_time = std::chrono::steady_clock::now();
            qDebug() << "检测执行时间: " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() << " ms";
        }
        catch (const rs2::error& e) {
            break;
            //std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        }
        catch (const std::exception& e) {
            break;
            //std::cerr << e.what() << std::endl;
        }
    }
    delete detector;
}

