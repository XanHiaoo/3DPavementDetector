#pragma once
#include "PavementDetector.h"
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>


class PavementDetectorImpl
{
public:
    void detect(const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, pcl::visualization::PCLVisualizer::Ptr viewer);
    void detectDefects(const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, pcl::visualization::PCLVisualizer::Ptr viewer);
    DetectionResult getResult();

private:
    DetectionResult detectionResult;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOrg;
};
