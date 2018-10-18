//
// Created by howstar on 18-6-6.
//

#include "cloudShow.h"

ros::Publisher cloud_show::pub_obs;
ros::Publisher cloud_show::pub_curb;
ros::Publisher cloud_show::pub_box;
ros::Publisher cloud_show::pub;

const string frame_id = "velodyne";

void cloud_show::init_pub()
{
    ros::NodeHandle nh;
    pub_obs = nh.advertise<sensor_msgs::PointCloud2>("/velo_process/obs", 1);
    // pub_curb = nh.advertise<sensor_msgs::PointCloud2>("/velo_process/curb_lines", 1);
    // pub_box = nh.advertise<visualization_msgs::MarkerArray>("/velo_process/MarkerArray", 10);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/velo_process/cloud", 1);
}

// visualization_msgs::Marker get_box_marker(const Tracker &tracker)
// {

//     pcl::PointXYZI min_point_AABB(tracker.FiguresPro[0]._pt_min);
//     pcl::PointXYZI max_point_AABB(tracker.FiguresPro[0]._pt_max);

//     visualization_msgs::Marker markerBox;
//     markerBox.header.frame_id = "world";
//     markerBox.action = visualization_msgs::Marker::ADD;
//     markerBox.type = visualization_msgs::Marker::LINE_STRIP;
//     markerBox.scale.x = 0.1;
//     markerBox.color.b = 0;
//     markerBox.color.g = 1;
//     markerBox.color.r = 1;
//     markerBox.color.a = 1;
//     markerBox.lifetime = ros::Duration(0.2);
//     geometry_msgs::Point p0, p1, p2, p3, p00, p01, p02, p03, p0_p01, p1_p01, pl2, pl3;
//     //four bottom corner point
//     p0.x = min_point_AABB.data[0];
//     p1.x = max_point_AABB.data[0];
//     p2.x = max_point_AABB.data[0];
//     p3.x = min_point_AABB.data[0];

//     p0.y = min_point_AABB.data[1];
//     p1.y = min_point_AABB.data[1];
//     p2.y = max_point_AABB.data[1];
//     p3.y = max_point_AABB.data[1];

//     p0.z = min_point_AABB.data[2];
//     p1.z = min_point_AABB.data[2];
//     p2.z = min_point_AABB.data[2];
//     p3.z = min_point_AABB.data[2];
//     //four up corner point
//     p00.x = min_point_AABB.data[0];
//     p01.x = max_point_AABB.data[0];
//     p02.x = max_point_AABB.data[0];
//     p03.x = min_point_AABB.data[0];

//     p00.y = min_point_AABB.data[1];
//     p01.y = min_point_AABB.data[1];
//     p02.y = max_point_AABB.data[1];
//     p03.y = max_point_AABB.data[1];

//     p00.z = max_point_AABB.data[2];
//     p01.z = max_point_AABB.data[2];
//     p02.z = max_point_AABB.data[2];
//     p03.z = max_point_AABB.data[2];

//     markerBox.points.push_back(p0);
//     markerBox.points.push_back(p1);
//     markerBox.points.push_back(p2);
//     markerBox.points.push_back(p3);
//     markerBox.points.push_back(p0);

//     markerBox.points.push_back(p00);
//     markerBox.points.push_back(p01);
//     markerBox.points.push_back(p02);
//     markerBox.points.push_back(p03);
//     markerBox.points.push_back(p00);

//     markerBox.points.push_back(p01);
//     markerBox.points.push_back(p1);
//     markerBox.points.push_back(p2);
//     markerBox.points.push_back(p02);
//     markerBox.points.push_back(p03);
//     markerBox.points.push_back(p3);

//     stringstream nsk;
//     nsk << tracker.ID << "box";
//     markerBox.ns = nsk.str();

//     return markerBox;
// }

void cloud_show::show_points(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_obs,
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_ori)
{
    // visualization_msgs::MarkerArray markerArray;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr curb_line(new pcl::PointCloud<pcl::PointXYZI>);
    // *curb_line += curbDetection.side_points_left_;
    // *curb_line += curbDetection.side_points_right_;

    // cloud_show::pub_box.publish(markerArray);

    sensor_msgs::PointCloud2 output2;
    pcl::toROSMsg(*cloud_obs, output2);
    output2.header.frame_id = frame_id;
    cloud_show::pub_obs.publish(output2);

    // sensor_msgs::PointCloud2 output3;
    // pcl::toROSMsg(*curb_line, output3);
    // output3.header.frame_id = frame_id;
    // cloud_show::pub_curb.publish(output3);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for(auto pt : cloud_ori->points)
    {
        cloud->push_back(pt);
    }

    sensor_msgs::PointCloud2 output4;
    pcl::toROSMsg(*cloud, output4);
    output4.header.frame_id = frame_id;
    cloud_show::pub.publish(output4);
}