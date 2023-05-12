#pragma once

#include "PavementDetector.h"
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>


class PavementDetectorImpl
{
public:
    void detect(const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, const  pcl::visualizer::PCLVisualizer::Ptr viewer);
    void detectDefects();
    DetectionResult getResult();

private:
    DetectionResult detectionResult;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOrg;
};
