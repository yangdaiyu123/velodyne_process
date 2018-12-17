#ifndef VELOPROCESS_SRC_OBSDETECTION_H_
#define VELOPROCESS_SRC_OBSDETECTION_H_

#include "stdafx.h"

class ObstacleDetection
{
  public:
    ObstacleDetection();
    ~ObstacleDetection();

    void detectObstacle(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, std::vector<int> &obs_idx);
};

#endif