//
// Created by howstar on 18-6-6.
//

#include "cloud_show.h"

ros::Publisher cloud_show::pub_obs;
ros::Publisher cloud_show::pub_curb;
ros::Publisher cloud_show::pub_box;
ros::Publisher cloud_show::pub;

const string frame_id = "rslidar";

void cloud_show::init_pub(ros::NodeHandle nh)
{
    pub_obs = nh.advertise<sensor_msgs::PointCloud2>("obs", 1);
    // pub_curb = nh.advertise<sensor_msgs::PointCloud2>("/curb_lines", 1);
    // pub_box = nh.advertise<visualization_msgs::MarkerArray>("/MarkerArray", 10);
    pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);
}

void cloud_show::show_points(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_obs,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    sensor_msgs::PointCloud2 output2;
    pcl::toROSMsg(*cloud_obs, output2);
    output2.header.frame_id = frame_id;
    cloud_show::pub_obs.publish(output2);

    sensor_msgs::PointCloud2 output4;
    pcl::toROSMsg(*cloud, output4);
    output4.header.frame_id = frame_id;
    cloud_show::pub.publish(output4);
}