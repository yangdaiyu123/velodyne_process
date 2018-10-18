#ifndef _ROAD_CURB_DETECTION_
#define _ROAD_CURB_DETECTION_

#include"stdafx.h"
#include"grid_obs_create.h"
#include"RansacLine.h"

#define MAX_DIS2 10000


class CurbDetection {

public:
	explicit CurbDetection(const pcl::PointCloud<pcl::PointXYZI>& grid_obs_points);
	~CurbDetection();

	void 
		detectCurb();

//	void
//		checkSpeedBump(GridCreator& obsGrid);
	//void
	//	prefencePath2Velo(const vector<gnssType::PosDataUnit>& posList);

private:
	void 
		findNearestObs();

	void
		dis_movement(int startidx, int endidx, pcl::PointXYZI& min_dis_l, pcl::PointXYZI& min_dis_r);

public:
//	vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> > side_points_left_;
//	vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> > side_points_right_;
	pcl::PointCloud<pcl::PointXYZI> side_points_left_;
    pcl::PointCloud<pcl::PointXYZI> side_points_right_;
	vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> > road_fitted_;

	vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> > pt_vec_left_;
	vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> > pt_vec_right_;

	vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> > pt_vec_way_;

	static kalmanInfo last_line_para_l;
	static kalmanInfo last_line_para_r;
	static kalmanInfo last_para_way;

	static int track_num_l_;
	static int track_num_r_;

	/*static*/ kalmanInfo line_para_l_;
	/*static*/ kalmanInfo line_para_r_;

	kalmanInfo para_way_;



private:

	//double* pos_velo_;
	//Eigen::Vector4d curb_para_;

    pcl::PointCloud<pcl::PointXYZI> _grid_obs_points;


};

#endif