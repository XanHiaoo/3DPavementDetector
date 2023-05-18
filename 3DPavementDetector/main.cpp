#include <pcl/visualization/pcl_visualizer.h>
#include <librealsense2/rs.hpp>
#include <filesystem>
#include <Windows.h>
#include <iostream>
#include <chrono>
#include <map>
#include "../3DPavementDetectorDll/PavementDetector.h"
using namespace std;

std::map<DETECTION_RESULT_CODE, std::string> codeMap = {
    {DETECTION_OK, "DETECTION_OK"},
    {DETECTION_NG, "DETECTION_NG"},
    {DETECTION_ERROR, "DETECTION_ERROR"},
    {DETECTION_STILL, "DETECTION_STILL"}
};

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

int main()
{
    // 初始化可视化器
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(1.0, 1.0, 1.0);
    viewer->setCameraPosition(0, 0, -2, 0, 0, 0);

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();
    PavementDetector* detector = new PavementDetector();
       
    while(true)
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();

        auto depth = frames.get_depth_frame();

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);

        auto cloud = points_to_pcl(points);
        detector->detect(cloud,viewer);

        viewer->spinOnce(33);
    }
    return 0;
}

