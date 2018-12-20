
#include "hazard_point_detect.h"

//using namespace hazard_detection;
//#include"hdl_grabber.h"
#define SHADOW_COUNT 20
#define WAVEANG_IGNORE 100
#define OBSSELF_STARTANG_DEGREE 65
#define OBSSELF_ENDANG_DEGREE 240
#define GROUNDHEIGHT_INLASER -0.1
#define PITCHFORWARD_DEG 1.32805
#define OBSSELF_DITANCE_M 1.7

using namespace pcl;

//calculate physical id
int get_physical_id(int pos)
{
    return (pos % 2 == 0) ? (pos / 2) : (pos / 2 + 16);
}
int get_logical_id(int pos)
{
    return (pos < 16) ? (pos * 2) : ((pos - 16) * 2 + 1);
}

//unsigned
//hdl_cloudIndex(int height, int width)
//{
//	int physicalid = get_logical_id(width);
//	return (unsigned)(height * kLidarLine + physicalid);
//}

unsigned
hdl_cloudIndex(int idx_sweep, int idx_beam, int num_sweep)
{
    int idx = idx_beam < 8 ? idx_beam : 23 - idx_beam;
    return (unsigned)(idx * num_sweep + idx_sweep);
}

double
calculateAlphaD(float d0, float d1, double angleR)
{
    double d0_2 = d0 * d0;
    double d1_2 = d1 * d1;
    double h = sqrt(d0_2 + d1_2 - 2 * d0 * d1 * cos(angleR));

    double alphaR = acos((d1_2 + h * h - d0_2) / (2 * h * d1));
    return alphaR;
}

HazardDetection::HazardDetection()
    : lidar_width_(kLidarLine)
{
}

HazardDetection::~HazardDetection()
{
}

void HazardDetection::detectHazardPoint(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_ptr, pcl::PointCloud<pcl::PointSrc>::ConstPtr cloudscr_ptr, vector<int> &resultIdx)
{
    lidar_height_ = cloud_ptr->size() / lidar_width_;

    std::vector<std::vector<double>> wavyAng_List(lidar_height_);
    std::vector<std::vector<double>> thetaAng_List(lidar_height_);

    double Del_Theta = 2 * M_PI / 180;

    for (unsigned i = 0; i < lidar_height_; i++)
    {
        std::vector<double> wavyAng(kLidarLine, WAVEANG_IGNORE);
        std::vector<double> thetaAng(kLidarLine);
        std::vector<bool> pTan2_Ang(kLidarLine, 0);
        for (unsigned j = 1; j < kLidarLineDown - 1; j++)
        {
            unsigned int index = hdl_cloudIndex(i, j, lidar_height_);
            unsigned int index_next = hdl_cloudIndex(i, j + 1, lidar_height_);
            unsigned int index_pre = hdl_cloudIndex(i, j - 1, lidar_height_);
            double L1 = cloudscr_ptr->points[index_pre].radius;
            double L2 = cloudscr_ptr->points[index].radius;
            double L3 = cloudscr_ptr->points[index_next].radius;
            PointXYZI P1 = cloud_ptr->points[index_pre];
            PointXYZI P2 = cloud_ptr->points[index];
            PointXYZI P3 = cloud_ptr->points[index_next];

            double dis_h = (P3.x - P1.x) * (P3.x - P1.x) + (P3.y - P1.y) * (P3.y - P1.y);
            double dis_v = (P3.z - P1.z) * (P3.z - P1.z);
            double k = dis_v / dis_h;
            if (k > 1)
                pTan2_Ang[j] = true;

            //self obstacle filtered by radius : distance = 0.0 or too close to car
            if (L1 < OBSSELF_DITANCE_M || L2 < OBSSELF_DITANCE_M || L3 < OBSSELF_DITANCE_M) //过滤车上的激光点干扰(必须三个都检测)
            {
                wavyAng[j] = WAVEANG_IGNORE;
            }
            else if (std::isnan(L1) || std::isnan(L2) || std::isnan(L3))
            {
                wavyAng[j] = WAVEANG_IGNORE;
            }
            /*	else if (shadowFlag[index])
                    {
                        wavyAng[j] = WAVEANG_IGNORE;
                    }*/
            // else if (P1.z < -3.5 || P2.z < -3.5 || P3.z < -3.5)//过滤水下的点
            // {
            // 	wavyAng[j] = WAVEANG_IGNORE;
            // }

            else
            {
                double /*delta_L1, delta_L2, */ theta1, theta2;

                theta1 = calculateAlphaD(L1, L2, Del_Theta);

                theta2 = calculateAlphaD(L3, L2, Del_Theta);

                wavyAng[j] = theta1 + theta2;
                thetaAng[j] = theta1;
            }
        }
        /////judgment for every face(i)
        float delt_Ang_thre = 360.0 / ANG_THRESHOLD_COUNT;
        float delt_Ang_Measure = 360.0 / (cloud_ptr->points.size() / kLidarLine);
        for (unsigned int j = 1; j < kLidarLineDown; j++)
        {
            if (abs(wavyAng[j] - WAVEANG_IGNORE) < 0.1 || abs(wavyAng[j + 1] - WAVEANG_IGNORE) < 0.1)
            {
                continue;
            }
            else
            {
                int threshold_index = delt_Ang_Measure * i / delt_Ang_thre;
                float threshold_positive0 = g_threshold_const_positive[j][threshold_index];
                float threshold_negative0 = g_threshold_const_negative[j][threshold_index];
                float threshold_positive1 = g_threshold_const_positive[j + 1][threshold_index];
                float threshold_negative1 = g_threshold_const_negative[j + 1][threshold_index];

                float angT0, angT1;
                // if (j < 10)
                // {
                //     angT0 = thetaAng[j] + velo_ang_rad[j];
                //     angT1 = thetaAng[j + 1] + velo_ang_rad[j + 1];
                // }
                // else
                // {
                angT0 = wavyAng[j] - M_PI;
                angT1 = wavyAng[j + 1] - M_PI;
                // }
                if (angT0 > threshold_positive0 && angT1 < threshold_negative1)
                {
                    //	cloud_obs_pm->points.push_back(cloud_ptr->points[hdl_cloudIndex(i, j)]);
                    //	cloud_obs->points.push_back(cloud_ptr->points[hdl_cloudIndex(i, j)]);
                    resultIdx.push_back(hdl_cloudIndex(i, j, lidar_height_));
                }
                else if (angT0 < threshold_negative0 && angT1 > threshold_positive1)
                {
                    //	cloud_obs_mp->points.push_back(cloud_ptr->points[hdl_cloudIndex(i, j + 1)]);
                    //cloud_obs->points.push_back(cloud_ptr->points[hdl_cloudIndex(i, j + 1)]);
                    resultIdx.push_back(hdl_cloudIndex(i, j + 1, lidar_height_));
                }
                // else if (abs(angT0) > threshold_positive0 &&
                // 		 abs(angT1) > threshold_positive1)
                // {
                // 	//cloud_obs->points.push_back(cloud_ptr->points[hdl_cloudIndex(i, j + 1)]);
                // 	resultIdx.push_back(hdl_cloudIndex(i, j + 1));
                // }
                else if (/*abs(angT0-M_PI)<10.0*M_PI/180 && */ pTan2_Ang[j])
                {
                    resultIdx.push_back(hdl_cloudIndex(i, j, lidar_height_));
                }
            }
        }
    }

    //	cloudobs_ptr = cloud_obs;
}
