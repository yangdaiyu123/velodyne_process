//
// Created by howstar on 18-6-6.
//

#ifndef PROJECT_CLOUDSHOW_H
#define PROJECT_CLOUDSHOW_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include "obs_cluster.h"
#include "road_curb_detection.h"

namespace cloud_show{

    extern ros::Publisher pub_obs;
    extern ros::Publisher pub_curb;
    extern ros::Publisher pub_box;
    extern ros::Publisher pub;

    void init_pub();

    void show_points(const CTrackersCenter& trackingCenter,const CurbDetection& curbDetection,
    std::vector<int> &obs_idx,pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_data);
    // void show_points(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_obs,
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ori);
}

#endif //PROJECT_CLOUDSHOW_H