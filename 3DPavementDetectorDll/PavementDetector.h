#pragma once
#include <vector>
#include <pcl/visualization/pcl_visualizer.h>

#ifdef PAVEMENTDETECTOR_EXPORTS
#define PAVEMENT_DETECTOR_API __declspec(dllexport)
#else
#define PAVEMENT_DETECTOR_API __declspec(dllimport)
#endif

#pragma warning(push)
#pragma warning(disable:4251)


enum DETECTION_RESULT_CODE : int
{
	DETECTION_ERROR = -1,
	DETECTION_OK = 0,
	DETECTION_NG = 1,
	DETECTION_STILL = 2
};

struct PAVEMENT_DETECTOR_API DetectionResult
{
	DETECTION_RESULT_CODE code = DETECTION_OK;
	int bulgeNum = 0;
	int HoleNum = 0;
};

class PavementDetectorImpl;

class PAVEMENT_DETECTOR_API PavementDetector
{
public:
	PavementDetector();
	~PavementDetector();
	void detect(const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, pcl::visualization::PCLVisualizer::Ptr viewer);
	DetectionResult getResult();

private:
	PavementDetectorImpl* detector;
};

#pragma warning(pop)