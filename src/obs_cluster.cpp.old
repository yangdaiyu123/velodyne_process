//
// Created by howstar on 18-7-13.
//

#include "obs_cluster.h"

//#include <unordered_set>

using namespace std;
using namespace pcl;


//float squareDistance(PointProp a,PointProp b){
//    return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
//}

ObsCluster::ObsCluster(const pcl::PointCloud<pcl::PointXYZI> &obs_points,
const CurbDetection &curbDetection)
:_point_ori(obs_points),
_curb_detection(curbDetection){

    if(_curb_detection.line_para_r_.pt_num==0){
        _curb_detection.line_para_r_.b_=20;
    }
    if(_curb_detection.line_para_l_.pt_num==0){
        _curb_detection.line_para_r_.b_=-20;
    }
    //对障碍物点建立KD树索引
    _kdtree.setInputCloud(obs_points.makeShared());

    //建立点属性列表,初始化每个点的属性
    for(int i=0;i<obs_points.size();i++){
        PointProp temp_pt;
        temp_pt.idx=i;
        temp_pt.pointType=noise;
        _point_prop_vec.push_back(temp_pt);
    }
}

bool ObsCluster::if_point_within_curb(pcl::PointXYZI pt) {
    float x1=pt.y*_curb_detection.line_para_l_.k_+_curb_detection.line_para_l_.b_-1.0f;
    float x2=pt.y*_curb_detection.line_para_r_.k_+_curb_detection.line_para_r_.b_+1.0f;
    return (pt.x-x1)*(pt.x-x2)<0;
}

vector<int> ObsCluster::getNeighbors(int search_idx,float Eps) {

    PointXYZI temp_pt=_point_ori[search_idx];

    vector<int> pointIdxRadiusSearch;
    vector<float> pointRadiusSquaredDistance;

    _kdtree.radiusSearch(temp_pt, Eps, pointIdxRadiusSearch, pointRadiusSquaredDistance);

    //因为每次第一个点都是搜索点本身,去掉这个点
    pointIdxRadiusSearch.erase(pointIdxRadiusSearch.begin());

    return pointIdxRadiusSearch;
}

void ObsCluster::DBSCAN(float Eps,int MinPts){

    for(int i=0;i<_point_ori.size();i++){

        if(_point_prop_vec[i].visited)
            continue;

        if(!if_point_within_curb(_point_ori[i]))
            continue;

        _point_prop_vec[i].visited=true;//将该点作为种子点,标记为已访问

        vector<int> neighbors=getNeighbors(i,Eps);

        _point_prop_vec[i].neighbor_pts_count=neighbors.size();

        if(neighbors.size()<MinPts+1){
            continue;
        }
        else{
            _point_prop_vec[i].pointType=core;
            expandCluster(i, neighbors, Eps, MinPts);//加入到新簇
        }
    }
}

void ObsCluster::expandCluster(int sp_idx,const vector<int>& search_cluster, float Eps, int MinPts) {

    GroupProp temp_group;
    temp_group._pts_inside_idx.push_back(sp_idx);

    //建立栈,初始化加入刚开始的几个聚类点
    stack<int> st_cluster;
    for(auto idx:search_cluster){
        st_cluster.push(idx);
        _point_prop_vec[idx].visited=true;
        temp_group._pts_inside_idx.push_back(idx);
    }

    while(!st_cluster.empty()){

        int pt_idx=st_cluster.top();
        st_cluster.pop();

        vector<int> neiPts_vec=getNeighbors(pt_idx,Eps);
        int neigh_size=neiPts_vec.size();
        _point_prop_vec[pt_idx].neighbor_pts_count=neigh_size;
        if(neigh_size>=MinPts+1)
            _point_prop_vec[pt_idx].pointType=core;
        else
            _point_prop_vec[pt_idx].pointType=border;

        for(auto tmp_idx:neiPts_vec) {

            if (_point_prop_vec[tmp_idx].visited) {
                continue;
            }

            st_cluster.push(tmp_idx);
            _point_prop_vec[tmp_idx].visited = true;
            temp_group._pts_inside_idx.push_back(tmp_idx);

        }
    }

    _group_vec.push_back(temp_group);

}

void ObsCluster::calGroupProp() {
    for(int i=0;i<_group_vec.size();i++){
        GroupProp &group=_group_vec[i];
        float cx=0,cy=0,cz=0;
        float min_x=_point_ori[group._pts_inside_idx[0]].x;
        float max_x=min_x;
        float min_y=_point_ori[group._pts_inside_idx[0]].y;
        float max_y=min_y;
        float min_z=_point_ori[group._pts_inside_idx[0]].z;
        float max_z=min_z;
        group._pt_num=group._pts_inside_idx.size();
        for(auto idx:group._pts_inside_idx){
            PointXYZI pt=_point_ori[idx];
            pt.intensity=(i+1)*10;//给同一簇的点赋同一个强度
            cx+=pt.x;
            cy+=pt.y;
            cz+=pt.z;
            if(min_x>pt.x) min_x=pt.x;
            if(max_x<pt.x) max_x=pt.x;
            if(min_y>pt.y) min_y=pt.y;
            if(max_y<pt.y) max_y=pt.y;
            if(min_z>pt.z) min_z=pt.z;
            if(max_z<pt.z) max_z=pt.z;
            group._pts_inside.push_back(pt);
        }
        group._center.x=cx/group._pt_num;
        group._center.y=cy/group._pt_num;
        group._center.z=cz/group._pt_num;
        group._pt_min.x=min_x;
        group._pt_min.y=min_y;
        group._pt_min.z=min_z;
        group._pt_max.x=max_x;
        group._pt_max.y=max_y;
        group._pt_max.z=max_z;
        group._width_x=max_x-min_x;
        group._length_y=max_y-min_y;
        group._height_z=max_z-min_z;
        group._radius=(group._width_x+group._length_y)/2.f;
    }
}



void ObsCluster::cluster() {

    DBSCAN(1,4);
    calGroupProp();

}


//------------------tracking---------------------//

float figureSimiCalculation(Tracker tracker, GroupProp figure)
{
    float simi = 0.0;
    float geoSimi = 0.0;
    int posSimi = 0;

    //Position Similarity -- integral part
    double distance = sqrt(pow(tracker.PropertiesKF._center.x - figure._center.x,2)
                          + pow(tracker.PropertiesKF._center.y - figure._center.y,2));
    if(distance > 8.0)
        posSimi = 0;
    else
        posSimi = round((8.0 - distance) * 10.0 / 8.0);//waiting test

    //Geometry Similarity -- decimal part
    float biggerRadius = max(tracker.PropertiesKF._radius, figure._radius);
    geoSimi = (biggerRadius -abs(tracker.PropertiesKF._radius - figure._radius)) / biggerRadius;

    //
    simi = posSimi + geoSimi;

    return simi;
}

void CTrackersCenter::inputSingFrameFigures(std::vector<GroupProp> figureList, int frameID,double cur_time)
{
    ////0.figureList initialize
    for(auto &figure:figureList)
    {
        figure.id = frameID;
    }
    ////0.trackerList initialize
    if(TrackerList.empty())
    {
        //waiting
        for(auto &figure:figureList)
        {
            Tracker trackerNewOne(getNewObjID());
            trackerNewOne.create_WithNewMember(figure);
            TrackerList.push_back(trackerNewOne);
            time_=cur_time;
        }
        return;
    }


    ////1.Bayes
    int numFigure = figureList.size();
    int numTracker = TrackerList.size();
    figureBelong.resize(numFigure,-1);
    std::vector<int> trackerStatus(numTracker,-1);//fill in code of figure the tracker taking |-1 = missing
    std::vector<int> trackerStatus_new(0);//fill in code of figure the tracker taking |-1 = missing

    if(!figureList.empty()) {
        //1.1 similarity calculation
        Eigen::MatrixXd Matsimi = Eigen::MatrixXd::Zero(numTracker, numFigure);
        for(unsigned itracker = 0 ; itracker < TrackerList.size() ; itracker++)
        {
            for(unsigned jfigure = 0 ; jfigure < figureList.size() ; jfigure++)
            {
                //simi definition //waiting
                Matsimi(itracker, jfigure) = figureSimiCalculation(TrackerList[itracker], figureList[jfigure]);
            }
        }

        //1.2belong
        //find max simi in col(different tracker)
        std::vector<float> figureSimi(numFigure);
        for(unsigned ifigure = 0; ifigure < figureList.size() ; ifigure++)
        {
            float maxSimi = 0.0;
            int codeTracker = -1;
            for(unsigned jtracker = 0 ; jtracker < TrackerList.size() ; jtracker++)
            {
                if(maxSimi < Matsimi(jtracker, ifigure))
                {
                    maxSimi = Matsimi(jtracker, ifigure);
                    codeTracker = jtracker;
                }
            }
            figureBelong[ifigure] = codeTracker;
            figureSimi[ifigure] = maxSimi;
        }
        //solve multi figure match the same tracker
        for(unsigned ifigure = 0; ifigure < figureBelong.size() ; ifigure++)
        {
            for(unsigned jfigure = 1; jfigure < figureBelong.size() ; jfigure++)
            {
                if(figureBelong[ifigure] == figureBelong[jfigure])
                {
                    if(figureSimi[ifigure] > figureSimi[jfigure])
                    {
                        figureBelong[jfigure] = -1;
                    }
                    if(figureSimi[ifigure] < figureSimi[jfigure])
                    {
                        figureBelong[ifigure] = -1;
                    }
                }
            }
        }
        //get the tracker status
        for(unsigned ifigure = 0 ; ifigure < figureBelong.size() ; ifigure++)
        {
            if(figureBelong[ifigure] >= 0)
            {
                trackerStatus[figureBelong[ifigure]] = ifigure;
            }
        }

        //cal velocity of tracked figure
        for(int i=0;i<trackerStatus.size();i++){
            if(trackerStatus[i]==-1)
                continue;

            double time_span=cur_time-time_;
            GroupProp track_one=TrackerList[i].PropertiesKF;
            GroupProp figure_one=figureList[trackerStatus[i]];
            double dis_x=figure_one._center.x-track_one._center.x;
            double dis_y=figure_one._center.y-track_one._center.y;
            TrackerList[i].velocity_x=dis_x/time_span;
            TrackerList[i].velocity_y=dis_y/time_span;
        }
    }

    ////2.comb list
    //create new tracker with new figure
    trackerStatus_new = trackerStatus;
//    TrackerList.clear();
    for(unsigned ifigure = 0 ; ifigure < figureBelong.size() ; ifigure++)
    {
        if(figureBelong[ifigure] == -1)
        {
            Tracker newOne(getNewObjID());
            TrackerList.push_back(newOne);
            trackerStatus_new.push_back(ifigure);
        }
    }


    std::cout<<"tracker size is "<<TrackerList.size()<<std::endl;

    //update with new figure
    //update even if no figure
    if(figureBelong.size()>0) {
        for (unsigned i = 0; i < TrackerList.size(); i++) {
            //update
            if (trackerStatus_new[i] == -1) {
                TrackerList[i].update_WithNoMember();
            } else if (figureBelong[trackerStatus_new[i]] == -1) {
                int idx=trackerStatus_new[i];
                if(idx>=figureList.size()) continue;
                TrackerList[i].create_WithNewMember(figureList[idx]);
            } else {
                int idx=trackerStatus_new[i];
                if(idx>=figureList.size())
                    continue;
                TrackerList[i].update_WithNewMember(figureList[idx]);
            }

        }
    }

    //delete missing tracker
    TrackerList_swap.resize(0);
    for(auto track:TrackerList)
    {
        if(frameID - track.getLastFnumber() <= 5)
            TrackerList_swap.push_back(track);
    }
    TrackerList.swap(TrackerList_swap);

    for(int i=0;i<figureBelong.size();i++){
        int idx=figureBelong[i];
        if(idx==-1) continue;

        TrackerList_swap[idx]._pts_inside=TrackerList_swap[idx].FiguresPro[0]._pts_inside;
        for(auto &pt:TrackerList_swap[idx]._pts_inside){
            pt.intensity=(TrackerList_swap[idx].ID%10)*10;
        }
    }

//    for(auto &group:TrackerList){
//        group._pts_inside=group.FiguresPro[0]._pts_inside;
//        for(auto &pt:group._pts_inside){
//            pt.intensity=(group.ID%10)*10;
//        }
//    }

    //time_=cur_time;
    ////show

}

void Tracker::create_WithNewMember(const GroupProp &figure)
{
    for(unsigned i = 0 ; i < FIGURE_RESERVE ; i++)
    {
        FiguresPro[i] = figure;
    }
    PropertiesKF = figure;

    ////KF
    //KF.inputMeasurement_first(figure.center.x, figure.center.y);
    //KF.outputCorrection(PropertiesKF.center.x, PropertiesKF.center.y);
    countMeb_total = 1;

}

void Tracker::update_WithNewMember(const GroupProp &figure)
{
    for(unsigned i = FIGURE_RESERVE - 1 ; i > 0 ; i--)
    {
        FiguresPro[i] = FiguresPro[i - 1];
    }
    FiguresPro[0] = figure;
    PropertiesKF._radius = figure._radius;

    //KF
    if(countMeb_total == 1)
    {
        KF.inputMeasurement_first(figure._center.x, figure._center.y, figure._center.x - FiguresPro[1]._center.x, figure._center.y - FiguresPro[1]._center.y);
        PropertiesKF = figure;
    }
    else
    {
        KF.inputMeasurement(figure._center.x, figure._center.y);
        KF.outputCorrection(PropertiesKF._center.x, PropertiesKF._center.y);
    }
    countMeb_total ++;

    //update length,width and height
    PropertiesKF._width_x=(FiguresPro[0]._width_x+FiguresPro[1]._width_x*(countMeb_total-1))/countMeb_total;
    PropertiesKF._length_y=(FiguresPro[0]._length_y+FiguresPro[1]._length_y*(countMeb_total-1))/countMeb_total;
    PropertiesKF._height_z=(FiguresPro[0]._height_z+FiguresPro[1]._height_z*(countMeb_total-1))/countMeb_total;
}

void Tracker::update_WithNoMember()
{
    KF.inputMeasurement(PropertiesKF._center.x, PropertiesKF._center.y);
    KF.outputCorrection(PropertiesKF._center.x, PropertiesKF._center.y);
}

int Tracker::getLastFnumber()
{
    return FiguresPro[0].id;
}

int CTrackersCenter::getNewObjID()
{
    trackerIDCount++;
    if (trackerIDCount > 50)
    {
        trackerIDCount = 0;
    }
    return trackerIDCount;
}