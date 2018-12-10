#ifndef HAZARD_POINT_DETECT_H_
#define HAZARD_POINT_DETECT_H_

//#include"BaseType.h"
#include "stdafx.h"
// #include "hdl_grabber.h"
#include <vector>
using namespace std;

class HazardDetection 
{
public:
	HazardDetection();
	~HazardDetection();

	void 
		detectHazardPoint(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, pcl::PointCloud<pcl::PointSrc>::ConstPtr cloud_src, vector<int>& resultidx);


private:
	int lidar_width_;
	int lidar_height_;

};

#endif