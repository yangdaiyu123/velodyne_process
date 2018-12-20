#include "obs_detection_by_altitude.h"

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

ObstacleDetection::ObstacleDetection()
:cloud_road_(new pcl::PointCloud<PointXYZIndex>)
{
}

ObstacleDetection::~ObstacleDetection()
{
}

void ObstacleDetection::detectObstacle(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, std::vector<int>& obs_idx)
{
    //高度过滤
    // pcl::PointCloud<pcl::PointXYZI> cloud_road;
    for (int i = 0; i < cloud->size(); i++)
    {
        auto one = cloud->points[i];
        PointXYZIndex temp_pt;
        temp_pt.x = one.x;
        temp_pt.y = one.y;
        temp_pt.z = one.z;
        transformXYZToIndex(temp_pt);
        if (abs(one.z) < 0.3)
        {
            cloud_road_->push_back(temp_pt);
            obs_idx.push_back(i);
        }
    }

    // //将过滤后的点云再做ransac平面提取
    // //创建分割时所需要的模型系数对象coefficients及存储内点的点索引集合对象inliers。
    // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // //创建分割对象
    // pcl::SACSegmentation<pcl::PointXYZI> seg;
    // //可选择配置，设置模型系数需要优化
    // seg.setOptimizeCoefficients(true);
    // //必须配置，设置分割的模型类型、所用的随机参数估计方法、距离阈值、输入点云
    // seg.setModelType(pcl::SACMODEL_SPHERE);
    // seg.setMethodType(pcl::SAC_RRANSAC);
    // seg.setDistanceThreshold(0.2);
    // seg.setInputCloud(cloud_road.makeShared());
    // //引发分割实现，并存储分割结果到点集合inliers及存储平面模型的系数coefficients
    // seg.segment(*inliers, *coefficients);

    // for(auto one : inliers->indices)
    // {
    //     cloud_obs->push_back(cloud_road[one]);
    // }
}

void ObstacleDetection::transformXYZToIndex(PointXYZIndex &pt)
{
    float width_i = pt.x + grid_width_num_ / 2;
    float height_j = grid_length_num_ / 2 - pt.y;
    pt.row = height_j / grid_resolution_;
    pt.col = width_i / grid_resolution_;
}

void growRoad()
{
}