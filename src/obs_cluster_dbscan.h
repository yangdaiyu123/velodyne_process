//
// Created by howstar on 18-7-13.
//

#ifndef PROJECT_OBS_CLUSTER_H
#define PROJECT_OBS_CLUSTER_H

#include "stdafx.h"

#include <pcl/kdtree/kdtree_flann.h>

#include "kalmanFilter.h"

enum PointType
{
    noise,
    border,
    core
};

class PointProp
{
  public:
    int idx;

    PointType pointType = noise; //1 noise 2 border 3 core

    int neighbor_pts_count = 0; //the number of points in Eps radiu

    bool visited = false;
    PointProp() {}
};



struct GroupProp
{
    float _width_x;
    float _length_y;
    float _height_z;

    pcl::PointXYZI _pt_min; //矩形框(x,y,z)最小的点
    pcl::PointXYZI _pt_max; //矩形框(x,y,z)最大的点

    pcl::PointXYZI _center;
    float _radius;

    std::vector<int> _pts_inside_idx;
    pcl::PointCloud<pcl::PointXYZI> _pts_inside;
    //    std::vector<PointProp> _pts_prop;

    int _pt_num;

    int id;

    // Object _object;
};

class ObsCluster
{

  public:
    // ObsCluster(const pcl::PointCloud<pcl::PointXYZI>& obs_points,
    // const CurbDetection &curbDetection);
    ObsCluster(const pcl::PointCloud<pcl::PointXYZI> &obs_points);

    void cluster();

    std::vector<GroupProp> group_vec() const { return _group_vec; };

    pcl::PointCloud<pcl::PointXYZI>::Ptr pts_cluster() const { return _pts_cluster; };

  private:
    std::vector<GroupProp> _group_vec;

    std::vector<PointProp> _point_prop_vec;

    pcl::PointCloud<pcl::PointXYZI> _point_ori;
    // CurbDetection _curb_detection;

    pcl::KdTreeFLANN<pcl::PointXYZI> _kdtree;

    pcl::PointCloud<pcl::PointXYZI>::Ptr _pts_cluster;

  private:
    void expandCluster(int sp_idx, const std::vector<int> &search_cluster, float Eps, int MinPts);

    void DBSCAN(float Eps, int MinPts);

    std::vector<int> getNeighbors(int search_idx, float Eps);

    void calGroupProp();

    // bool if_point_within_curb(pcl::PointXYZI pt);
};

class TrackerObs
{
  public:
    TrackerObs()
    {
        countMeb_total = 0;
        countMeb_total = 0;
        velocity_x = 0;
        velocity_y = 0;
        ID = 0;
    }
    explicit TrackerObs(int idIn)
    {
        ID = idIn;
        velocity_x = 0;
        velocity_y = 0;
        countMeb_total = 0;
    }

  private:
    static const int FIGURE_RESERVE = 10;

  public:
    //variable
    GroupProp FiguresPro[FIGURE_RESERVE]; //CONSTANT_FIGURERESERVE 10

    GroupProp PropertiesKF;

    pcl::PointCloud<pcl::PointXYZI> _pts_inside;

    int ID; //id of figure

    kalmanFilter KF;

    int countMeb_total; //tracked times

    double velocity_x;
    double velocity_y;

    //function
    void create_WithNewMember(const GroupProp &figure);

    void update_WithNewMember(const GroupProp &figure);

    void update_WithNoMember();

    int getLastFnumber();
};

class TrackersCenter
{
  public:
    TrackersCenter()
    {
        trackerIDCount = 0;
        time_ = 0.0;
    };

    //function
  public:
    void inputSingFrameFigures(std::vector<GroupProp> figureList, int frameID, double cur_time);

    std::vector<TrackerObs> getTrackerList() const { return TrackerList; }

  private:
    int getNewObjID();

    //variable
  public:
    //fill in code of TrackerObs the figure belong to|-1 = new TrackerObs
    std::vector<int> figureBelong;

    std::vector<TrackerObs> TrackerList_swap;

  private:
    int trackerIDCount;

    std::vector<TrackerObs> TrackerList;

    double time_;
};

#endif //PROJECT_OBS_CLUSTER_H
