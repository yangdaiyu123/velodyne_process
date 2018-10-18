//
// Created by howstar on 18-6-9.
//

#ifndef PROJECT_STDAFX_H
#define PROJECT_STDAFX_H

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#define VELO_LINE 32
#define VELO_LINE_DONW 23
#define ANG_THRESHOLD_COUNT 3600

// struct PointSrc{
//     float _radius;
//     float _angle;

//     PointSrc(){
//         _radius=0.f;
//         _angle=0.f;
//     }

//     PointSrc(float radius,float angle){
//         _radius=radius;
//         _angle=angle;
//     }
// };

extern double VELO_ANG_32[32];
extern double g_LiDAR_pos[6];
extern double velo_ang_rad[32];

extern std::vector<float> g_threshold_const_positive[VELO_LINE_DONW + 1]; //点云障碍物阈值，正
extern std::vector<float> g_threshold_const_negative[VELO_LINE_DONW + 1]; //点云障碍物阈值，负

//extern std::string g_file_dir;

void
filterThresholdOfObstacle(double laserHeightM, double laserPitchD, double addConstant,
                          double mulConstant, double obsHeightThred);

//extern int off_idx[4];

inline int get_idx(int idx_beam,int idx_sweep){
    //(pos<16) ? (pos * 2) : ((pos - 16) * 2 + 1);
    idx_beam=(idx_beam<16)?idx_beam*2:(idx_beam-16)*2+1;
    return idx_sweep*VELO_LINE+idx_beam;
}

//inline int get_idx_with_offset(int idx_sweep,int idx_beam,int num_sweep){
//    int idx=idx_sweep+off_idx[idx_beam%4];
//    if(idx>num_sweep) idx-=num_sweep;
//    return get_idx(idx,idx_beam);
//}

void readCaliFile(std::string path);

pcl::PointXYZI
transport_point(pcl::PointXYZI pts);

#endif //PROJECT_STDAFX_H

