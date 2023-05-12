#include "PavementDetectorImpl.h"
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/boost.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>
#include <pcl/common/common.h>
#include <chrono>
#include <pcl/common/transforms.h>


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

pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::VoxelGrid<pcl::PointXYZ> sor_voxel;
    sor_voxel.setInputCloud(cloud);
    sor_voxel.setLeafSize(0.01f, 0.01f, 0.01f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    sor_voxel.filter(*cloud_filtered);

    if (cloud->size() == cloud_filtered->size()) {
        sor_voxel.setLeafSize(0.5f, 0.5f, 0.5f);
        sor_voxel.filter(*cloud_filtered);
    }

    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_outlier;
    sor_outlier.setInputCloud(cloud_filtered);
    sor_outlier.setMeanK(100);
    sor_outlier.setStddevMulThresh(0.5);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_removal(new pcl::PointCloud<pcl::PointXYZ>);
    sor_outlier.filter(*cloud_removal);

    return cloud_removal;
}

void segmentCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& inlier_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& outlier_cloud, float& sum_z) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    inlier_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    outlier_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    sum_z = 0.0;

    for (int i = 0; i < cloud->size(); i++) {
        pcl::PointXYZ point = cloud->at(i);
        if (std::find(inliers->indices.begin(), inliers->indices.end(), i) != inliers->indices.end()) {
            // Point is inliers (belongs to plane)
            inlier_cloud->push_back(point);
            sum_z += point.z;
        }
        else {
            // Point is outlier (not part of plane)
            outlier_cloud->push_back(point);
        }
    }
}

void euclideanClusterExtraction(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,const float cluster_tolerance,const int min_cluster_size,const int max_cluster_size,std::vector<pcl::PointIndices>& cluster_indices)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);

    ec.extract(cluster_indices);
}

void visualizeConvexHulls(const std::vector<pcl::PointIndices>& cluster_indices, const pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_cloud, pcl::visualization::PCLVisualizer::Ptr viewer, float avg_z_threshold) {
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(1.0, 1.0, 1.0, v1);
    // Compute the convex hull for each cluster
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        float avg_z = 0.0;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cluster_cloud->points.push_back(outlier_cloud->points[*pit]);
            avg_z += outlier_cloud->points[*pit].z;
        }
        avg_z /= it->indices.size();

        float r, g, b;
        if (avg_z > avg_z_threshold) {
            // 绿色
            r = 0.0;
            g = 1.0;
            b = 0.0;
        }
        else {
            // 红色
            r = 1.0;
            g = 0.0;
            b = 0.0;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ConvexHull<pcl::PointXYZ> chull;
        chull.setInputCloud(cluster_cloud);
        chull.setDimension(3);
        std::vector<pcl::Vertices> polygons;
        chull.reconstruct(*convex_hull, polygons);

        // Compute the surface area of the convex hull
        float surface_area = 0.0;
        for (size_t i = 0; i < polygons.size(); ++i)
        {
            pcl::PointCloud<pcl::PointXYZ> cloud_convex_hull;
            for (size_t j = 0; j < polygons[i].vertices.size(); ++j)
            {
                cloud_convex_hull.push_back((*convex_hull)[polygons[i].vertices[j]]);
            }
            float area = pcl::calculatePolygonArea(cloud_convex_hull);
            surface_area += area;
        }
        std::cout << "Convex hull surface area: " << surface_area << std::endl;

        // Add the convex hull and cluster to the visualization
        if (surface_area > 0.01) {
            viewer->addPointCloud<pcl::PointXYZ>(convex_hull, "convex_hull_" + std::to_string(it - cluster_indices.begin()),v1);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.5, 1.0, "convex_hull_" + std::to_string(it - cluster_indices.begin()));
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "convex_hull_" + std::to_string(it - cluster_indices.begin()));
            viewer->addPointCloud<pcl::PointXYZ>(cluster_cloud, "cluster_" + std::to_string(it - cluster_indices.begin()),v1);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, "cluster_" + std::to_string(it - cluster_indices.begin()));
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cluster_" + std::to_string(it - cluster_indices.begin()));
            viewer->addText3D(avg_z < avg_z_threshold ? "hole" : "bulge" + std::to_string(it - cluster_indices.begin()), pcl::PointXYZ(cluster_cloud->points[0].x, cluster_cloud->points[0].y, cluster_cloud->points[0].z + 0.1), 0.03, 1.0, 0.5, 1.0, "text_" + std::to_string(it - cluster_indices.begin()),v1);
        }
    }
}

void PavementDetectorImpl::detect(const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,pcl::visualization::PCLVisualizer::Ptr viewer)
{
    cloudOrg = inputCloud;
    detectDefects(inputCloud,viewer);
}

void PavementDetectorImpl::detectDefects(const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, pcl::visualization::PCLVisualizer::Ptr viewer)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud = cloudOrg;

    //去除NAN和极大值
    removeNaNAndInfFromPointCloud(cloud);

    //点云降采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_filtered = filterPointCloud(cloud);
 
    //离群点去除
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_removal(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_removal = removeOutliers(cloud_filtered);

    //最大平面分割
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    
    float sum_z = 0.0;
    float avg_z_threshold = 1.0;
    segmentCloud(cloud_removal, inlier_cloud, outlier_cloud, avg_z_threshold);

    if (inlier_cloud->empty()) {
        std::cout << "Inlier cloud is empty, cannot compute average z value" << std::endl;
    }
    else {
        avg_z_threshold = sum_z / inlier_cloud->size();
        std::cout << "Inlier cloud average z value: " << avg_z_threshold << std::endl;
    }

    //平面外点云聚类
    std::vector<pcl::PointIndices> cluster_indices;
    euclideanClusterExtraction(outlier_cloud, 0.05, 100, 100000, cluster_indices);

    //显示聚类
    visualizeConvexHulls(cluster_indices, outlier_cloud, viewer, avg_z_threshold);

    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(1.0, 1.0, 1.0, v2);
    /*viewer->addPointCloud<pcl::PointXYZ>(inlier_cloud, "inliers");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "inliers",v2); */
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud",v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "cloud", v2);
}

DetectionResult PavementDetectorImpl::getResult()
{
    detectionResult = DetectionResult();   
    return detectionResult;
}
