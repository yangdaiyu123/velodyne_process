#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
//#include<pcl/range_image/range_image.h>

#include "road_curb_detection.h"
#include "hazard_point_detect.h"
#include "grid_obs_create.h"
#include "cloudShow.h"
#include "obs_cluster.h"

CTrackersCenter trackersCenter;
int g_frame_num = 0;

using namespace std;
using namespace pcl;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{
    //    ros::Rate loop_rt(10);
    double t1 = pcl::getTime();
    // Create a container for the data.
    // sensor_msgs::PointCloud2 output;

    //transfer msg to PointCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_data(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input, *cloud_data);

    if (cloud_data->size() % VELO_LINE != 0)
    {
        cout << "WARNING:The number of point clouds is not an integer multiple of 32!" << endl;
        //return;
    }

    cloud_data->width = VELO_LINE;
    cloud_data->height = cloud_data->size() / VELO_LINE;

    // Do data processing here...
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_swap(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<PointSrc>::Ptr cloud_src(new pcl::PointCloud<PointSrc>);
    for (int i = 0; i < cloud_data->size(); i++)
    {
        pcl::PointXYZI temp_pt = cloud_data->points[i];
        temp_pt=transform_point(temp_pt);
        float x = temp_pt.x;
        float y = temp_pt.y;
        float z = temp_pt.z;
        float radius = sqrt(x * x + y * y + z * z);
        float angle = float(atan2(y, x) / M_PI * 180);
        pcl::PointSrc tmp;
        tmp.radius=radius;
        tmp.angle=angle;
        cloud_src->push_back(tmp);
        cloud_swap->push_back(temp_pt);
    }
    std::cout << "transform time is " << pcl::getTime() - t1 << std::endl;

    vector<int> obs_idx;
    HazardDetection hazardDetection;
    hazardDetection.detectHazardPoint(cloud_swap, cloud_src, obs_idx);

    GridCreator grid_obs;
    grid_obs.createGrid(cloud_swap, obs_idx);

    CurbDetection curb_detection(grid_obs.grid_obs_points_);
    curb_detection.detectCurb();

    ObsCluster obsCluster(grid_obs.grid_obs_points_, curb_detection);
    obsCluster.cluster();
    trackersCenter.inputSingFrameFigures(obsCluster.group_vec(), g_frame_num, pcl::getTime());

    //show
    cloud_show::show_points(trackersCenter, curb_detection, obs_idx, cloud_swap);

    g_frame_num++;

    double t2 = pcl::getTime();
    std::cout << "total time is " << t2 - t1 << std::endl;

    //    loop_rt.sleep();
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "velo_process");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Create a ROS subscriber for the input point cloud
//    ros::Subscriber sub = nh.subscribe("/rfans_driver/rfans_points", 1, cloud_cb);
//    ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, cloud_cb);
    ros::Subscriber sub=nh.subscribe("/rslidar_points",1,cloud_cb);

    // Create a ROS publisher for the output point cloud
    //    pub = nh.advertise<std_msgs::String>("/velo_process/img", 1);

    //readCaliFile(g_file_dir+"Testdata-001-Calib/Testdata-001-HDL32-E.txt");

    //calculate obstacle _angle threshold, run only once
    filterThresholdOfObstacle(0.0, 0.0, 0, 0, 0.1);

    cloud_show::init_pub(private_nh);

    // Spin
    ros::spin();
}
