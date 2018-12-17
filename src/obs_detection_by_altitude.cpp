#include "obs_detection_by_altitude.h"

ObstacleDetection::ObstacleDetection()
{

}

ObstacleDetection::~ObstacleDetection()
{
    
}

void ObstacleDetection::detectObstacle(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, std::vector<int> &obs_idx)
{
    for(int i=0;i<cloud->size();i++)
    {
        auto temp_pt=cloud->points[i];
        if(abs(temp_pt.z)>0.1)
        {
            obs_idx.push_back(i);
        }
    }
}