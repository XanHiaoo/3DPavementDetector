#include "PavementDetector.h"
#include "PavementDetectorImpl.h"
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

PavementDetector::PavementDetector()
{
	detector = new PavementDetectorImpl();
}

PavementDetector::~PavementDetector()
{
	delete detector;
}

void PavementDetector::detect(const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, pcl::visualization::PCLVisualizer::Ptr viewer)
{
	detector->detect(inputCloud, viewer);
}

DetectionResult PavementDetector::getResult()
{
	return detector->getResult();
}