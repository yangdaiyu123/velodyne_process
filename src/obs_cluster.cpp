//
// Created by howstar on 18-6-5.
//
#include "stdafx.h"
// #include "CObstaclePair.h"
#include "obs_cluster.h"
// #include "cloudShow.h"
//#include <pcl/features/moment_of_inertia_estimation.h>
// #include "GrahamScan.h"

using namespace std;
using namespace pcl;

// ros::Publisher pub_group;

const double neboANDsideDistanceThreshold[] = {
        0.80,0.80,0.81,0.83,0.85,//5
        0.87,0.90,0.93,0.97,0.91,//5
        0.98,0.94,0.94,0.96,0.92,//5
        1.00,1.00,1.00,1.00,1.00,1.00,1.00
        ,1.00
        ,1.00
        ,1.00
        ,1.00
        ,1.00
        ,1.00
        ,1.00
        ,1.00
        ,1.00
        ,1.00
};


//template < typename T>
//vector< size_t>  sort_indexes(const vector< T>  & v) {
//
//    // initialize original index locations
//    vector< size_t>  idx(v.size());
//    for (size_t i = 0; i != idx.size(); ++i) idx[i] = i;
//
//    // sort indexes based on comparing values in v
//    sort(idx.begin(), idx.end(),
//         [& v](size_t i1, size_t i2) {return v[i1] <  v[i2];});
//
//    return idx;
//}
//CObstaclePair::CObstaclePair()
//{
//
//}
CObstaclePair::~CObstaclePair()
{

}

// bool CObstaclePair::if_point_within_curb(pcl::PointXYZI pt){
//     float x1=pt.y*m_line_para[0]+m_line_para[1]+2.0f;
//     float x2=pt.y*m_line_para[2]+m_line_para[3]-2.0f;
//     return (pt.x-x1)*(pt.x-x2)<0;
// }

CObstaclePair::CObstaclePair(pcl::PointCloud<pcl::PointXYZI>::Ptr ori_xyz
        , pcl::PointCloud<pcl::PointSrc>::Ptr ori_scr)
//                             const CurbDetection& curbDetection)
//:m_curb(curbDetection)
{

    //initialize
    m_ori_src = ori_scr;
    m_ori_xyz = ori_xyz;
    num_sweep=m_ori_xyz->size()/32;

//    m_curb=curbDetection;

    // pcl::PointCloud<pcl::PointXYZI>::Ptr laserPointCloud[32];
    // pcl::PointCloud<PointSrc>::Ptr laserPointCloud_scr[32];
    std::vector<std::vector<std::pair<unsigned, unsigned > > > Start_sidePairFeature(32);
    std::vector<std::vector<std::pair<unsigned, unsigned> > > End_sidePairFeature(32);

    // //1.split 32 lines
    // for(unsigned ibeam_count = 0 ; ibeam_count < 32 ; ibeam_count++)
    // {
    //     pcl::PointCloud<pcl::PointXYZI>::Ptr beam_laserPointCloud(new pcl::PointCloud<pcl::PointXYZI>);
    //     pcl::PointCloud<PointSrc>::Ptr beam_laserPointCloud_scr(new pcl::PointCloud<PointSrc>);
    //     for(unsigned jface_count = 0 ; jface_count < num_sweep ; jface_count++)
    //     {
    //         beam_laserPointCloud->points.push_back(ori_xyz->points[get_idx(ibeam_count,jface_count)]);
    //         beam_laserPointCloud_scr->points.push_back(ori_scr->points[get_idx(ibeam_count,jface_count)]);
    //     }
    //     laserPointCloud[ibeam_count] = beam_laserPointCloud;
    //     laserPointCloud_scr[ibeam_count] = beam_laserPointCloud_scr;
    // }


    //2.pair feature extraction
    ///2.1 feature judgment
    for(unsigned ibeam_count = 0 ; ibeam_count < 23 ; ibeam_count++ )
    {
        //2.1.1 find side and nebo
        vector<float> diffArray(num_sweep);
        vector<bool> isSideFlagList(num_sweep);
        vector<bool> isNeboFlagList(num_sweep);

        int neboHood = 15;
        //neighbor judgment
        for(unsigned i = 0 ; i < num_sweep ; i++)
        {
            float depth0 = m_ori_src->points[get_idx(ibeam_count,i)].radius;
            float depth1 = depth0;
            unsigned j = 0;
            if(depth0 < 1.5*500)
            {
                continue;
            }
            else
            {
                //ignore radius = 0.00 or radius < threshold
                for(unsigned m = 1 ; m < neboHood ; m++)
                {
                    unsigned mcount1 = i + m;
                    if(mcount1 >= 0 && mcount1 < num_sweep)
                    {
                        depth1 = m_ori_src->points[get_idx(ibeam_count,mcount1)].radius;
                        if(depth1 > 1.5*500)
                        {
                            j = mcount1;
                            break;
                        }
                    }
                }
                //
                if(depth1 > 1.5*500 && fabs(depth0 - depth1) > neboANDsideDistanceThreshold[ibeam_count] * 0.35*500&&
                    fabs(i-j)>5/*THRE_NEBODISMAX*/ )
                {
                    if(depth0 < depth1)
                    {
                        isSideFlagList[i] = true;
                        isNeboFlagList[j] = true;
                    }
                    else
                    {
                        isNeboFlagList[i] = true;
                        isSideFlagList[j] = true;
                        //featureCloud[icount * m_numFace + i].obsNebo = true;
                        //featureCloud[icount * m_numFace + j].obsSide = true;
                    }
                    //featureCloud[icount * m_numFace + j].obsNeboDis = (int)(depth1 - depth0) + 100;
                    //featureCloud[icount * m_numFace + i].obsNeboDis = (int)(depth0 - depth1) + 100;
                }
            }
        }

        //2.1.2 make side pair
        std::vector<std::pair<unsigned,unsigned> > start_sidelist;
        std::vector<std::pair<unsigned,unsigned> > end_sidelist;

        for(unsigned i_index = 0 ; i_index < num_sweep ; i_index++)
        {
            unsigned j_index = i_index;

            std::pair<unsigned,unsigned> startside;
            std::pair<unsigned,unsigned> endside;
            startside.first = ibeam_count;
            endside.first = ibeam_count;

            if(isSideFlagList[i_index])
            {
                for(unsigned j = 1 ; j < 1500 && (j+i_index) < num_sweep ; j++)
                {
                    j_index = i_index + j;
                    if(isNeboFlagList[j_index])
                    {
                        break;
                    }
                    else if(isSideFlagList[j_index])
                    {
                        startside.second = i_index;
                        endside.second = j_index;
                        pcl::PointXYZI startpt = m_ori_xyz->points[get_idx(ibeam_count,i_index)];
                        pcl::PointXYZI endpt = m_ori_xyz->points[get_idx(ibeam_count,j_index)];

                        double distance_2 = pow((startpt.x - endpt.x), 2) + pow((startpt.y - endpt.y), 2);
                        //distance between start and end can not be too long
                        //points between start and end can not be too few
                        if(distance_2 < 36.00 && endside.second - startside.second > 3)
                        {
                            start_sidelist.push_back(startside);
                            end_sidelist.push_back(endside);
                        }
                        break;
                    }
                }
                i_index = j_index;
            }
        }
        Start_sidePairFeature[ibeam_count] = start_sidelist;
        End_sidePairFeature[ibeam_count] = end_sidelist;
    }


    FeaturePair_Grouping(Start_sidePairFeature,End_sidePairFeature);
}

void CObstaclePair::FeaturePair_Grouping(std::vector<std::vector<std::pair<unsigned, unsigned> > > Start_sidePairFeature,
                                         std::vector<std::vector<std::pair<unsigned, unsigned> > > End_sidePairFeature) {
    //inputData
    //std::vector<std::pair<unsigned, unsigned>> Start_sidePairFeature[32];
    //std::vector<std::pair<unsigned, unsigned>> End_sidePairFeature[32];


    //resData
    std::vector<Tracking_group> groupList(0);
    std::vector<Tracking_group> groupList_swap(0);

    ////1.group similar segments into a group
    //traverse segment
    for (unsigned ibeam = 0; ibeam < 32; ibeam++) {
        if (Start_sidePairFeature[ibeam].size() == 0)
            continue;

        for (unsigned jface = 0; jface < Start_sidePairFeature[ibeam].size(); jface++) {
            bool findGroup = false;
            for (unsigned kgroup = 0; kgroup < groupList.size(); kgroup++) {
                //similarities calculation
                trackingGroupMemb groupMembNew;
                groupMembNew.start_index = Start_sidePairFeature[ibeam][jface];
                groupMembNew.end_index = End_sidePairFeature[ibeam][jface];
                trackingGroupMemb groupCur = groupList[kgroup].Gmember[groupList[kgroup].Gmember.size() - 1];

                if (groupCur.start_index.first == groupMembNew.start_index.first)
                    continue;

                int curCode_s = get_idx( groupCur.start_index.first,groupCur.start_index.second);
                int curCode_e = get_idx( groupCur.end_index.first,groupCur.end_index.second);
                int newCode_s = get_idx( groupMembNew.start_index.first,groupMembNew.start_index.second);
                int newCode_e = get_idx( groupMembNew.end_index.first,groupMembNew.end_index.second);
                double delt_distance = abs(m_ori_src->points[curCode_s].radius - m_ori_src->points[newCode_s].radius)
                                       + abs(m_ori_src->points[curCode_e].radius - m_ori_src->points[newCode_e].radius);
                double delt_angstart = abs(m_ori_src->points[curCode_s].angle - m_ori_src->points[newCode_s].angle);
                double delt_angend = abs(m_ori_src->points[curCode_e].angle - m_ori_src->points[newCode_e].angle);
                double max_sang = max(m_ori_src->points[curCode_s].angle, m_ori_src->points[newCode_s].angle);
                double min_eang = min(m_ori_src->points[curCode_e].angle, m_ori_src->points[newCode_e].angle);


                //push back a segment to a existed group
                if (delt_distance < 500.0 && (delt_angstart < 500.0 || delt_angend < 500.0 || (min_eang - max_sang) > 0)) {
                    groupList[kgroup].Gmember.push_back(groupMembNew);
                    findGroup = true;
                    break;
                }
            }
            if (!findGroup) {
                //creat a new group with a segment
                Tracking_group groupNew;
                trackingGroupMemb groupMembNew;
                groupMembNew.start_index = Start_sidePairFeature[ibeam][jface];
                groupMembNew.end_index = End_sidePairFeature[ibeam][jface];

                if ((groupMembNew.end_index.second - groupMembNew.start_index.second) < 5 &&
                    groupMembNew.end_index.first < 18)
                    continue;

                groupNew.Gmember.push_back(groupMembNew);
                groupList.push_back(groupNew);
            }
        }
    }

    ////2.calculate properties of groups
    vector<Tracking_group> group_swap(0);
    for (unsigned igroup = 0; igroup < groupList.size(); igroup++) {
        if(GroupPropertiesCal(groupList[igroup])){
            // *m_obs_cloud+=*groupList[igroup].Gproperties.cloudInside;
            // if (if_point_within_curb(groupList[igroup].Gproperties.center))
            // {
                group_swap.push_back(groupList[igroup]);
            // }
        }
    }
    groupList.swap(group_swap);

    ////2.merge group
//    evaluate properties
    bool ifMerge = false;
    for (unsigned icount = 0; icount < 5 && (ifMerge || icount == 0); icount++)
        ifMerge = GroupMerge(groupList);
//    GroupMerge_xhd(groupList);


    ////3.filter group List
    groupList_swap.resize(0);
//    ros::Rate r(1);
    for (unsigned igroup = 0; igroup < groupList.size(); igroup++) {
        //
        if (groupList[igroup].Gmember.size() < 2)
            continue;

//        float width = 4.0;
//        if (abs(groupList[igroup].Gproperties.center.x) > width)
//            continue;

        groupList_swap.push_back(groupList[igroup]);
        //group_properities_swap.push_back(group_properities_swap[igroup]);

    }
    groupList.swap(groupList_swap);

    //cout << "group size is " << groupList.size() << endl;


    m_groupList_res = groupList;

}

pcl::PointCloud<pcl::PointXYZI>::Ptr CObstaclePair::GroupPointCloudColl(Tracking_group group) {

    pcl::PointCloud<pcl::PointXYZI>::Ptr Agroup_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    //collect all points belong to a group
    for (unsigned jmemb = 0; jmemb < group.Gmember.size(); jmemb++) {
        int count = group.Gmember[jmemb].end_index.second - group.Gmember[jmemb].start_index.second;
        for (unsigned kindex = 0; kindex <= count; kindex++) {
            int ptIndex = get_idx(group.Gmember[jmemb].start_index.first,
                    group.Gmember[jmemb].start_index.second + kindex);

            if (m_ori_src->points[ptIndex].radius > 1.0) {
//                if (if_point_within_curb(m_ori_xyz->points[ptIndex])) {
                    Agroup_cloud->push_back(m_ori_xyz->points[ptIndex]);
//                }
            }
        }
    }
    return Agroup_cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
CObstaclePair::get_group_box(pcl::PointCloud<pcl::PointXYZI>::Ptr group_points) {
    int num = group_points->size();
    pcl::PointXYZI l_pt,d_pt,r_pt,t_pt;
    float left=100;
    float right=-100;
    float top=-100;
    float down=100;
    //int l_idx,r_idx,t_idx,d_idx;
    for (int i = 0; i < num; i++) {
        pcl::PointXYZI tmp=group_points->points[i];
        if(left>tmp.x){
            left=tmp.x;
            //l_idx=i;
            l_pt=tmp;
        }
        if(right<tmp.x){
            right=tmp.x;
            //r_idx=i;
            r_pt=tmp;
        }
        if(top<tmp.y){
            top=tmp.y;
            //t_idx=i;
            t_pt=tmp;
        }
        if(down>tmp.y){
            down=tmp.y;
            //d_idx=i;
            d_pt=tmp;
        }

    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr box=get_line(l_pt,t_pt);
    *box+=*(get_line(t_pt,r_pt));
    *box+=*(get_line(r_pt,d_pt));
    *box+=*(get_line(d_pt,l_pt));

    return box;
}

bool dis_compare(const pair<int,double>& a,const pair<int,double>& b) {
    return a.second>b.second;
}


bool CObstaclePair::GroupPropertiesCal(Tracking_group &group) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointsIn(GroupPointCloudColl(group));
    group.Gproperties.cloudInside = pointsIn;

    if (pointsIn->empty())
        return false;

    //
    float cx = 0.0, cy = 0.0, cz = 0.0, radius = 0.0;
    float xMin = pointsIn->points[0].x, xMax = xMin, yMin = pointsIn->points[0].y, yMax = yMin;
    float zMin = pointsIn->points[0].z, zMax = zMin;

    for (unsigned ipoint = 0; ipoint < pointsIn->points.size(); ipoint++) {
        float x = pointsIn->points[ipoint].x;
        float y = pointsIn->points[ipoint].y;
        float z = pointsIn->points[ipoint].z;

        if (abs(x) < 1.5 && abs(y) < 1.5)
            continue;

        xMin = x < xMin ? x : xMin;
        xMax = x > xMax ? x : xMax;
        yMin = y < yMin ? y : yMin;
        yMax = y > yMax ? y : yMax;
        zMin = z < zMin ? z : zMin;
        zMax = z > zMax ? z : zMax;

        cx += x;
        cy += y;
        cz += z;
    }
    cx /= pointsIn->points.size();
    cy /= pointsIn->points.size();
    cz /= pointsIn->points.size();

//    vector<double> dis_vec;
//    for(int i=0;i<pointsIn->points.size();i++){
//        double dis=sqrt(pow(pointsIn->points[i].x-cx,2)+pow(pointsIn->points[i].y-cy,2));
//        dis_vec.push_back(dis);
//    }
    vector<pair<int, double> > dis_map;
    for (int i = 0; i < pointsIn->points.size(); i++) {
        double dis = sqrt(pow(pointsIn->points[i].x - cx, 2) + pow(pointsIn->points[i].y - cy, 2));
        dis_map.push_back(pair<int,double>(i,dis));
    }

//    vector<size_t> idx_vec=sort_indexes<double>(dis_vec);
    sort(dis_map.begin(), dis_map.end(),dis_compare);
    vector<pair<int, double> >::iterator iter = dis_map.begin();

    double dis_threshold = iter->second * 0.9;

//    vector<int> int_idx;
    for (; iter != dis_map.end(); iter++) {

        group.Gproperties.cloudInside->erase(group.Gproperties.cloudInside->begin() + iter->first);

        if (iter->second < dis_threshold)
            break;
    }
//    cx = (xMax + xMin) / 2.0f;
//    cy = (yMax + yMin) / 2.0f;
//    cz = (zMax + zMin) / 2.0f;
    if ((xMax - xMin) > (yMax - yMin)) {
        radius = (xMax - xMin) / 2.0f;
    } else {
        radius = (yMax - yMin) / 2.0f;
    }

    if (radius < 0.5) {
        radius = 0.5;
    }

    group.Gproperties.center.x = cx;
    group.Gproperties.center.y = cy;
    group.Gproperties.center.z = cz;
    group.Gproperties.radius = radius;
    group.Gproperties.width_x = xMax - xMin;
    group.Gproperties.length_y = yMax - yMin;
    group.Gproperties.height_z = zMax - zMin;
    group.Gproperties.density = 1.0 * pointsIn->size() / radius;
    stringstream nsk;
    nsk << "ns" << rand();
    group.Gproperties.id = nsk.str();

    return true;

}

bool CObstaclePair::GroupMerge(std::vector<Tracking_group> &groupList) {
    bool ifMergeAgroup = false;

    std::vector<Tracking_group> groupList_swap(0);
    for (unsigned igroup = 0; igroup < groupList.size(); igroup++) {
        GroupPropertiesCal(groupList[igroup]);
    }
    for (unsigned igroup = 0; igroup < groupList.size(); igroup++) {
        bool findGroup = false;
        for (unsigned jgroup = 0; jgroup < groupList_swap.size(); jgroup++) {
            //calculate overlap of two groups
            float disAB, deltradiusAB_max;
            disAB = sqrt(pow(groupList[igroup].Gproperties.center.x - groupList[jgroup].Gproperties.center.x, 2)
                         + pow(groupList[igroup].Gproperties.center.y - groupList[jgroup].Gproperties.center.y, 2));
//            deltradiusAB_max = max(groupList[igroup].Gproperties.radius, groupList[jgroup].Gproperties.radius);
            deltradiusAB_max = max(groupList[igroup].Gproperties.radius, groupList[jgroup].Gproperties.radius);
//            float delt_raduisAB_min=min(groupList[igroup].Gproperties.radius, groupList[jgroup].Gproperties.radius);
            if (deltradiusAB_max - disAB > -1.0 && groupList[igroup].Gproperties.density > 10 &&
                groupList[jgroup].Gproperties.density > 10)
//            if(deltradiusAB_max>disAB)
            {
                //calculate radius acce
                Tracking_group tryMerged_group;
                for (unsigned kmemb = 0; kmemb < groupList[igroup].Gmember.size(); kmemb++) {
                    tryMerged_group.Gmember.push_back(groupList[igroup].Gmember[kmemb]);
                }
                for (unsigned kmemb = 0; kmemb < groupList_swap[jgroup].Gmember.size(); kmemb++) {
                    tryMerged_group.Gmember.push_back(groupList_swap[jgroup].Gmember[kmemb]);
                }
                GroupPropertiesCal(tryMerged_group);
                float radius_acce = (tryMerged_group.Gproperties.radius - deltradiusAB_max) / deltradiusAB_max;

                if (radius_acce < 0.2) {
//                    //merge two groups into one
                    groupList_swap[jgroup] = tryMerged_group;
                    findGroup = true;
                    ifMergeAgroup = true;
                    break;
                }
            }
        }
        if (!findGroup) {
            groupList_swap.push_back(groupList[igroup]);
        }
    }
    groupList.swap(groupList_swap);

    return ifMergeAgroup;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr CObstaclePair::get_line(pcl::PointXYZI p1, pcl::PointXYZI p2) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    float width=(p2.x-p1.x);
    float height=(p2.y-p1.y);
    int num=20;
    float step_x=width/num;
    float step_y=height/num;
    for(int i=0;i<num;i++)
    {
        pcl::PointXYZI tmp;
        tmp.x=p1.x+step_x*i;
        tmp.y=p1.y+step_y*i;
        tmp_ptr->push_back(tmp);
    }
    return tmp_ptr;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr CObstaclePair::get_lines(pcl::PointCloud<pcl::PointXYZI>::Ptr convex_hull_pts) {
    int nums=convex_hull_pts->size();
    pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
    if(nums==4) {
        for (int i = 0; i < nums - 1; i++) {
            *output += *(get_line(convex_hull_pts->points[i], convex_hull_pts->points[i + 1]));
        }
        *output += *(get_line(convex_hull_pts->points[nums - 1], convex_hull_pts->points[0]));
    }
    else if(nums==8){
        for (int i = 0; i < 3; i++) {
            *output += *(get_line(convex_hull_pts->points[i], convex_hull_pts->points[i + 1]));
        }
        *output += *(get_line(convex_hull_pts->points[3], convex_hull_pts->points[0]));
        for (int i = 4; i < 7; i++) {
            *output += *(get_line(convex_hull_pts->points[i], convex_hull_pts->points[i + 1]));
        }
        *output += *(get_line(convex_hull_pts->points[7], convex_hull_pts->points[4]));
        for (int i = 0; i < 4; i++) {
            *output += *(get_line(convex_hull_pts->points[i], convex_hull_pts->points[i + 4]));
        }
    }
    return output;
}

bool CObstaclePair::GroupMerge_xhd(std::vector<Tracking_group> &groupList) {
    bool if_merge_group=false;

    std::vector<Tracking_group> group_merged;
    while (if_merge_group){
        for(int i=0;i<groupList.size();i++) {
            Tracking_group tmp_group=groupList[i];
            for(int j=0;j<group_merged.size();j++){
                //calculate overlap of two groups
                float disAB, deltradiusAB;
                disAB = sqrt(pow(tmp_group.Gproperties.center.x - groupList[j].Gproperties.center.x,2)
                             + pow(tmp_group.Gproperties.center.y - groupList[j].Gproperties.center.y,2));
//            deltradiusAB = max(groupList[igroup].Gproperties.radius, groupList[jgroup].Gproperties.radius);
                deltradiusAB = min(tmp_group.Gproperties.radius, groupList[j].Gproperties.radius);
                if(deltradiusAB - disAB > -0.0 /*&& groupList[igroup].Gproperties.density > 10 && groupList[jgroup].Gproperties.density > 10*/) {
                    //calculate radius acce
                    Tracking_group tryMerged_group;
                    for (unsigned kmemb = 0; kmemb < tmp_group.Gmember.size(); kmemb++) {
                        tryMerged_group.Gmember.push_back(tmp_group.Gmember[kmemb]);
                    }
                    for (unsigned kmemb = 0; kmemb < group_merged[j].Gmember.size(); kmemb++) {
                        tryMerged_group.Gmember.push_back(group_merged[j].Gmember[kmemb]);
                    }
                    GroupPropertiesCal(tryMerged_group);

                    group_merged.push_back(tryMerged_group);
                }


            }
        }
    }
    groupList.swap(group_merged);
}

//grouping end
//------------------------------------------------------------------------------------------------------------------//
//tracking

float figureSimiCalculation(Tracker tracker, Tracking_group figure)
{
    float simi = 0.0;
    float geoSimi = 0.0;
    int posSimi = 0;

    //Position Similarity -- integral part
    float distance = sqrt(pow(tracker.PropertiesKF.center.x - figure.Gproperties.center.x,2)
                          + pow(tracker.PropertiesKF.center.y - figure.Gproperties.center.y,2));
    if(distance > 8.0)
        posSimi = 0.0;
    else
        posSimi = (int)((8.0 - distance) * 10.0 / 8.0 + 0.5);//waiting test

    //Geometry Similarity -- decimal part
    float biggerRadius = max(tracker.PropertiesKF.radius, figure.Gproperties.radius);
    geoSimi = (biggerRadius -abs(tracker.PropertiesKF.radius - figure.Gproperties.radius)) / biggerRadius;

    //
    simi = posSimi + geoSimi;

    return simi;
}

void Tracker::create_WithNewMember(trackingGroupProp &figure)
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

void Tracker::update_WithNewMember(trackingGroupProp &figure)
{
    for(unsigned i = FIGURE_RESERVE - 1 ; i > 0 ; i--)
    {
        FiguresPro[i] = FiguresPro[i - 1];
    }
    FiguresPro[0] = figure;
    PropertiesKF.radius = figure.radius;

    //KF
    if(countMeb_total == 1)
    {
        KF.inputMeasurement_first(figure.center.x, figure.center.y, figure.center.x - FiguresPro[1].center.x, figure.center.y - FiguresPro[1].center.y);
        PropertiesKF = figure;
    }
    else
    {
        KF.inputMeasurement(figure.center.x, figure.center.y);
        KF.outputCorrection(PropertiesKF.center.x, PropertiesKF.center.y);
    }
    countMeb_total ++;

    //update length,width and height
    PropertiesKF.width_x=(FiguresPro[0].width_x+FiguresPro[1].width_x*(countMeb_total-1))/countMeb_total;
    PropertiesKF.length_y=(FiguresPro[0].length_y+FiguresPro[1].length_y*(countMeb_total-1))/countMeb_total;
    PropertiesKF.height_z=(FiguresPro[0].height_z+FiguresPro[1].height_z*(countMeb_total-1))/countMeb_total;
}

void Tracker::update_WithNoMember()
{
    KF.inputMeasurement(PropertiesKF.center.x, PropertiesKF.center.y);
    KF.outputCorrection(PropertiesKF.center.x, PropertiesKF.center.y);
}

long Tracker::getLastFnumber()
{
    return FiguresPro[0].Fnumber;
}

CTrackersCenter::CTrackersCenter()
:_obs_points(new pcl::PointCloud<pcl::PointXYZI>)
{
    trackerIDCount = 0;
}

CTrackersCenter::~CTrackersCenter()
{

}

long CTrackersCenter::getNewobjID()
{
    trackerIDCount++;
    if (trackerIDCount > 50)
    {
        trackerIDCount = 0;
    }
    return trackerIDCount;
}

void CTrackersCenter::inputSingFrameFigures(std::vector<Tracking_group> figureList, long frameID,double cur_time)
{
    _obs_points->clear();
    ////0.figureList initialize
    for(unsigned i = 0 ; i < figureList.size() ; i++)
    {
        figureList[i].Gproperties.Fnumber = frameID;
    }
    ////0.trackerList initialize
    if(TrackerList.size() == 0)
    {
        //waiting
        for(unsigned i = 0 ; i < figureList.size() ; i++)
        {
            Tracker trackerNewOne(getNewobjID());
            trackerNewOne.create_WithNewMember(figureList[i].Gproperties);
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
    if(figureList.size() != 0)
    {

        //1.1simi calculation
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
            trackingGroupProp track_one=TrackerList[i].PropertiesKF;
            trackingGroupProp figure_one=figureList[trackerStatus[i]].Gproperties;
            double dis_x=figure_one.center.x-track_one.center.x;
            double dis_y=figure_one.center.y-track_one.center.y;
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
            Tracker newOne(getNewobjID());
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
                TrackerList[i].create_WithNewMember(figureList[idx].Gproperties);
            } else {
                int idx=trackerStatus_new[i];
                if(idx>=figureList.size())
                    continue;
                TrackerList[i].update_WithNewMember(figureList[idx].Gproperties);
            }

        }
    }

    //delete missing tracker
    TrackerList_swap.resize(0);
    for(unsigned i = 0 ; i < TrackerList.size() ; i++)
    {
        if(frameID - TrackerList[i].getLastFnumber() <= 5)
        {
            for(auto one : *(TrackerList[i].FiguresPro[0].cloudInside))
            {
                one.intensity=(TrackerList[i].ID)%255;
                _obs_points->push_back(one);
            }
            TrackerList_swap.push_back(TrackerList[i]);
        }
    }
    TrackerList.swap(TrackerList_swap);

    time_=cur_time;
    ////show

}

