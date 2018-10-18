
#include "hazard_point_detect.h"

//using namespace hazard_detection;
//#include"hdl_grabber.h"
#define SHADOW_COUNT 20
#define OBSSELF_DITANCE_M 1.7*500
#define WAVEANG_IGNORE 100
#define OBSSELF_STARTANG_DEGREE 65
#define GROUNDHEIGHT_INLASER -0.1
#define PITCHFORWARD_DEG 1.32805


using namespace pcl;



//int get_idx(int i,int j){
//	int idx=j<VELO_LINE/2?j*2:(j-VELO_LINE/2)*2+1;
//	return i*VELO_LINE+idx;
//}

double 
calculateAlphaD(float d0, float d1, double angleR)
{
	double d0_2 = d0 * d0;
	double d1_2 = d1 * d1;
	double h = sqrt(d0_2 + d1_2 - 2 * d0*d1*cos(angleR));

	double alphaR = acos((d1_2 + h * h - d0_2) / (2 * h * d1));
	return alphaR;
}



HazardDetection::HazardDetection()
	:lidar_width_(VELO_LINE)
{
}

HazardDetection::~HazardDetection()
{
}

void 
HazardDetection::detectHazardPoint(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_ptr, pcl::PointCloud<pcl::PointSrc>::ConstPtr cloudscr_ptr, vector<int>& resultIdx)
{
	lidar_height_ = cloudscr_ptr->size() / lidar_width_;

	std::vector<std::vector<double> > wavyAng_List(lidar_height_);
	std::vector<std::vector<double> > thetaAng_List(lidar_height_);
	for (unsigned i = 0; i < lidar_height_; i++)
	{
		std::vector<double> wavyAng(VELO_LINE_DONW, WAVEANG_IGNORE);
		std::vector<double> thetaAng(VELO_LINE_DONW);
		std::vector<bool> pTan2_Ang(VELO_LINE_DONW, 0);
		for (unsigned j = 1; j < VELO_LINE_DONW-1; j++) {
            unsigned int index_pre = get_idx(i, j - 1);
            unsigned int index = get_idx(i, j);
            unsigned int index_next = get_idx(i, j + 1);
            double L1 = cloudscr_ptr->points[index_pre].radius;
            double L2 = cloudscr_ptr->points[index].radius;
            double L3 = cloudscr_ptr->points[index_next].radius;
            PointXYZI p1 = cloud_ptr->points[index_pre];
            PointXYZI p2 = cloud_ptr->points[index];
            PointXYZI p3 = cloud_ptr->points[index_next];

            //self obstacle filtered by _radius : distance = 0.0 or too close to car
            if (L1 < OBSSELF_DITANCE_M || L2 < OBSSELF_DITANCE_M ||
                L3 < OBSSELF_DITANCE_M)
            {
                wavyAng[j] = WAVEANG_IGNORE;
                continue;
            }

            if (p1.z < -3.5 || p2.z < -3.5 || p3.z < -3.5)//filter points under the water
            {
                wavyAng[j] = WAVEANG_IGNORE;
                continue;
            }

            double angle1 = cloudscr_ptr->points[index_pre].angle;
            double angle2 = cloudscr_ptr->points[index].angle;
            double angle3 = cloudscr_ptr->points[index_next].angle;


//            check if three points are in a line in plane of XOY
            if (abs(angle1 - angle2) > 1&&abs(angle1-angle2)<359 ||
                abs(angle2 - angle3) > 1&&abs(angle2-angle3)<359) {
                cout << "Warning:points order may be not right!" << endl;
                continue;
            }

            double dis_h = (p3.x - p1.x) * (p3.x - p1.x) + (p3.y - p1.y) * (p3.y - p1.y);
            double dis_v = (p3.z - p1.z) * (p3.z - p1.z);
            double k = dis_v / dis_h;
            if (k > 1)
                pTan2_Ang[j] = true;


            double /*delta_L1, delta_L2, */theta1, theta2;
            double Del_Theta = 1.3333 * M_PI / 180;

            theta1 = calculateAlphaD(L1, L2, Del_Theta);

            theta2 = calculateAlphaD(L3, L2, Del_Theta);

            wavyAng[j] = theta1 + theta2;
            thetaAng[j] = theta1;

        }

		//judgment for every face(i)
		float delt_Ang_thre = 360.0 / ANG_THRESHOLD_COUNT;
		float delt_Ang_Measure = 360.0 / (cloud_ptr->points.size() / VELO_LINE_DONW);
		for (unsigned int j = 1; j < VELO_LINE_DONW; j++)
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
				if (j < 10)
				{
					angT0 = thetaAng[j] + velo_ang_rad[j];
					angT1 = thetaAng[j + 1] + velo_ang_rad[j + 1];
				}
				else
				{
					angT0 = wavyAng[j] - M_PI;
					angT1 = wavyAng[j + 1] - M_PI;
				}
				if (angT0 > threshold_positive0 && angT1 < threshold_negative1)
				{
					//	cloud_obs_pm->points.push_back(cloud_ptr->points[get_idx(i, j)]);
					//	cloud_obs->points.push_back(cloud_ptr->points[get_idx(i, j)]);
					resultIdx.push_back(get_idx(i, j));
				}
				else if (angT0 < threshold_negative0 && angT1 > threshold_positive1)
				{
					//	cloud_obs_mp->points.push_back(cloud_ptr->points[get_idx(i, j + 1)]);
						//cloud_obs->points.push_back(cloud_ptr->points[get_idx(i, j + 1)]);
					resultIdx.push_back(get_idx(i, j + 1));
				}
				else if (abs(angT0) > threshold_positive0 &&
					abs(angT1) > threshold_positive1)
				{
					//cloud_obs->points.push_back(cloud_ptr->points[get_idx(i, j + 1)]);
					resultIdx.push_back(get_idx(i, j + 1));
				}
				else if (/*abs(angT0-M_PI)<10.0*M_PI/180 && */pTan2_Ang[j])
				{
					resultIdx.push_back(get_idx(i, j + 1));
				}

			}
		}

	}

	//	cloudobs_ptr = cloud_obs;
}


