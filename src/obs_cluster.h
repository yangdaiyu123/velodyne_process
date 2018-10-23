//
// Created by howstar on 18-6-5.
//

#ifndef PROJECT_COBSTACLEPAIR_H
#define PROJECT_COBSTACLEPAIR_H

#include "stdafx.h"
#include "hdl_grabber.h"
#include "kalmanFilter.h"
//#include "curb_detection.h"

enum Object{ Unknown=0,Pedestrian=1, Truck=2, Car=3  };

struct trackingGroupMemb{
    std::pair<unsigned,unsigned> start_index;
    std::pair<unsigned,unsigned> end_index;
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inPoints;
};

struct trackingGroupProp{
    float radius;
    float width_x;
    float length_y;
    float height_z;
    pcl::PointXYZI center;
    pcl::PointXY weightedCenter;
    float density;

    long Fnumber;

    std::string id;

    Object flag_object;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInside;
};

struct Tracking_group{
    std::vector<trackingGroupMemb> Gmember;
    trackingGroupProp Gproperties;
};


//input the original point cloud xyzi registered,
//output the pair side of obstacles
//       the inside points of obstacles
class CObstaclePair
{
public:
//    CObstaclePair();
//    CObstaclePair(pcl::PointCloud<pcl::PointXYZI>::Ptr ori_xyz, pcl::PointCloud<PointSrc>::Ptr ori_scr,
//    const CurbDetection& curbDetection);
    CObstaclePair(pcl::PointCloud<pcl::PointXYZI>::Ptr ori_xyz
            , pcl::PointCloud<pcl::PointSrc>::Ptr ori_scr);
    ~CObstaclePair();

    //function
public:
    std::vector<Tracking_group> group_list() const {return m_groupList_res;}

    pcl::PointCloud<pcl::PointXYZI>::Ptr
    get_lines(pcl::PointCloud<pcl::PointXYZI>::Ptr convex_hull_pts);

private:

    void FeaturePair_Grouping(std::vector<std::vector<std::pair<unsigned, unsigned> > > Start_sidePairFeature,
                              std::vector<std::vector<std::pair<unsigned, unsigned> > > End_sidePairFeature);

    bool GroupMerge(std::vector<Tracking_group> &groupList);

    bool GroupMerge_xhd(std::vector<Tracking_group> &groupList);

    pcl::PointCloud<pcl::PointXYZI>::Ptr GroupPointCloudColl(Tracking_group group);

    bool GroupPropertiesCal(Tracking_group &group);

    pcl::PointCloud<pcl::PointXYZI>::Ptr
    get_group_box(pcl::PointCloud<pcl::PointXYZI>::Ptr group_points);

    pcl::PointCloud<pcl::PointXYZI>::Ptr
    get_line(pcl::PointXYZI p1,pcl::PointXYZI p2);

    // bool if_point_within_curb(pcl::PointXYZI pt);

    //variable

private:

    int num_sweep;

    pcl::PointCloud<pcl::PointXYZI>::Ptr m_ori_xyz;

    pcl::PointCloud<pcl::PointSrc>::Ptr m_ori_src;

    std::vector<Tracking_group> m_groupList_res;

    // std::vector<float> m_line_para;
//    CurbDetection m_curb;


};


class Tracker
{
public:
    Tracker(){countMeb_total = 0; countMeb_total = 0;velocity_x=0;velocity_y=0;}
    Tracker(long idIn){ID = idIn;velocity_x=0;velocity_y=0;}
    ~Tracker(){}

private:
    static const int FIGURE_RESERVE=10;

public:
    //variable
    trackingGroupProp FiguresPro[FIGURE_RESERVE];//CONSTANT_FIGURERESERVE 10

    trackingGroupProp PropertiesKF;

    long ID;//id of figure

    kalmanFilter KF;

    int countMeb_total;//tracked times

    double velocity_x;
    double velocity_y;

    //function
    void create_WithNewMember(trackingGroupProp &figure);

    void update_WithNewMember(trackingGroupProp &figure);

    void update_WithNoMember();

    long getLastFnumber();

};

class CTrackersCenter{
public:
    CTrackersCenter();
    ~CTrackersCenter();

    //function
public:
    void inputSingFrameFigures(std::vector<Tracking_group> figureList, long frameID,double cur_time);

    std::vector<Tracker> getTrackerList() const {return TrackerList;}

    pcl::PointCloud<pcl::PointXYZI>::Ptr obs_points() const {return _obs_points;}

private:
    long getNewobjID();

    //variable
public:
    //fill in code of tracker the figure belong to|-1 = new tracker
    std::vector<int> figureBelong;

    std::vector<Tracker> TrackerList_swap;

private:
    long trackerIDCount;

    std::vector<Tracker> TrackerList;

    double time_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr _obs_points;
};

float figureSimiCalculation(Tracker tracker, Tracking_group figure);

#endif //PROJECT_COBSTACLEPAIR_H