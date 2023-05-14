#pragma once
#include "PavementDetector.h"
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>


class PavementDetectorImpl
{
public:
    void detect(const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);
    void detectDefects(const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);
    DetectionResult getResult();

private:
    DetectionResult detectionResult;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOrg;

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> bulge;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> hole;

    std::vector<float> bulgeArea;
    std::vector<float> holeArea;

    void getRoadDiseases(std::vector<pcl::PointIndices>& cluster_indices, const pcl::PointCloud<pcl::PointXYZ>::Ptr& outlier_cloud, float avg_z_threshold);
};
