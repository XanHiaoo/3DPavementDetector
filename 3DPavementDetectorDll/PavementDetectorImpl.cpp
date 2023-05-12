#include "PavementDetectorImpl.h"
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>


using namespace std;

void removeNaNAndInfFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::vector<int> indices;
    // 去除NaN
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    // 去除无穷大的值
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        if (std::isinf(cloud->points[i].x) ||
            std::isinf(cloud->points[i].y) ||
            std::isinf(cloud->points[i].z)) {
            cloud->points.erase(cloud->points.begin() + i);
            --i;
        }
    }
}

void PavementDetectorImpl::detect(const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,const  pcl::visualization::PCLVisualizer::Ptr viewer)
{
    cloudOrg = inputCloud;
    detectDefects();
}

void PavementDetectorImpl::detectDefects()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud = cloudOrg;
    removeNaNAndInfFromPointCloud(cloud);


    
}

DetectionResult PavementDetectorImpl::getResult()
{
    detectionResult = DetectionResult();   
    return detectionResult;
}
