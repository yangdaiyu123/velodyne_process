#include"road_curb_detection.h"

#include "ransac_line.h"

using namespace Eigen;
using namespace pcl;

int CurbDetection::track_num_l_ = 0;
int CurbDetection::track_num_r_ = 0;
kalmanInfo CurbDetection::last_para_way = kalmanInfo();
kalmanInfo CurbDetection::last_line_para_l = kalmanInfo();
kalmanInfo CurbDetection::last_line_para_r = kalmanInfo();

void 
CurbDetection::dis_movement(int startidx,int endidx,PointXYZI& min_dis_l, PointXYZI& min_dis_r)
{
	pcl::PointXYZI endPoint, startPoint;
	endPoint = road_fitted_[endidx];
	startPoint = road_fitted_[startidx];
	double maxY = max(endPoint.y, startPoint.y);
	double minY = min(endPoint.y, startPoint.y);
	double deltaY = endPoint.y - startPoint.y;

	vector<double> disVec_r(0);
	vector<double> disVec_l(0);
	vector<int> disVecIdx_r(0);
	vector<int> disVecIdx_l(0);
	for (int i=0;i<_grid_obs_points.size();i++)
	{
		pcl::PointXYZI tmp = _grid_obs_points[i];
		double x = tmp.x, y = tmp.y;
		if (y > minY&&y < maxY)
		{
			double dis = abs(x);
			if (x < 0)
			{
				disVec_r.push_back(dis);
				disVecIdx_r.push_back(i);
			}
			else
			{
				disVec_l.push_back(dis);
				disVecIdx_l.push_back(i);
			}
		}
	}
	if (disVec_l.size() > 0)
	{
//		auto min_dis_lp = std::min_element(std::begin(disVec_l), std::end(disVec_l));
        vector<double>::iterator min_dis_lp=std::min_element(disVec_l.begin(),disVec_l.end());
		int midx = min_dis_lp-disVec_l.begin();
		min_dis_l = _grid_obs_points[disVecIdx_l[midx]];
	}

	if (disVec_r.size() > 0)
	{
//		auto min_dis_rp= std::min_element(std::begin(disVec_r), std::end(disVec_r));
        vector<double>::iterator min_dis_rp=std::min_element(disVec_r.begin(),disVec_r.end());
		int midx = min_dis_rp - disVec_r.begin();
		min_dis_r = _grid_obs_points[disVecIdx_r[midx]];
	}

}

//void CurbDetection::checkSpeedBump(GridCreator& obsGrid)
//{
//	int num_width = g_width_max - g_width_min;
//	int num_length = g_length_max - g_length_min;
//
//	float limitD, limitU;
//	float grid_l = line_para_l_.b_ - 2;
//	float grid_r = line_para_r_.b_ + 2;
//	limitD = min(min(grid_l, grid_r), -4.0f);
//	limitU = max(max(grid_l, grid_r), 4.0f);
//
//	int histo_length = sqrt(num_width * num_width + num_length * num_length);
//	int theta_range = 20;
//	vector<int> hough_histo(theta_range * histo_length, 0);
//	vector<vector<float> > hough_ptz_histo(theta_range * histo_length);
//	vector<vector<PointXY> > hough_idx_histo(theta_range * histo_length);
//	vector<int> candidate_pts_idx;
//
//	for (int idx_grid = 0; idx_grid < obsGrid_.grid_obstacle_flag_.size(); idx_grid++)
//	{
//		if (obsGrid_.grid_obstacle_flag_[idx_grid])
//		{
//			int rows = idx_grid / num_width;
//			int cols = idx_grid % num_width;
//			float x = g_width_unit * (cols + g_width_min) + g_center_x + g_width_unit / 2.0;
//			float y = g_length_unit * (rows + g_length_min) + g_center_y + g_length_unit / 2.0;
//			if (x > limitD && x < limitU)
//			{
//				PointXYZI temp;
//				temp.x = x;
//				temp.y = y;
//				candidate_pts_idx.push_back(idx_grid);
//				for (int j = 0; j < theta_range; j++)
//				{
//					double theta = (j + 90 - theta_range / 2) * M_PI / 180;
//					int r = cols * cos(theta) + rows * sin(theta)+0.5;
//					if (r < 0 || r >= histo_length)
//						continue;
//
//					int idx_histo = r * theta_range + j;
//					hough_histo[idx_histo]++;
//					PointXY tmp;
//					tmp.x = cols;
//					tmp.y = rows;
//					hough_idx_histo[idx_histo].push_back(tmp);
//					auto t = obsGrid.grids_pt_[idx_grid].points;
//					float mean_z = 0;
//					for (auto iter = t.begin(); iter != t.end(); iter++)
//					{
//						mean_z += iter->z;
//					}
//					mean_z /= t.size();
//					hough_ptz_histo[idx_histo].push_back(mean_z);
//
//				}
//			}
//
//		}
//	}
//
//	auto max_iter = max_element(hough_histo.begin(), hough_histo.end());
//	int max_idx = max_iter - hough_histo.begin();
//	int max_theta = max_idx % theta_range + 90 - theta_range / 2;
//	int max_r = max_idx / theta_range;
//	vector<float> ptz_candidate = hough_ptz_histo[max_idx];
//	vector<PointXY> idx_candidate = hough_idx_histo[max_idx];
//	//vector<PointXY> idx_candidate;
//	int ptz_size = ptz_candidate.size();
//	if (ptz_size < 3)
//		return;
//
//	vector<float> ptz_candidate_new;
//	vector<PointXY> idx_candidate_new;
//	//int error = 0;
//	for (int idx_grid = 0; idx_grid < obsGrid_.grid_obstacle_flag_.size(); idx_grid++)
//	{
//		if (obsGrid_.grid_obstacle_flag_[idx_grid])
//		{
//			int rows = idx_grid / num_width;
//			int cols = idx_grid % num_width;
//			float x = g_width_unit * (cols + g_width_min) + g_center_x + g_width_unit / 2.0;
//			float y = g_length_unit * (rows + g_length_min) + g_center_y + g_length_unit / 2.0;
//			if (x > limitD && x < limitU)
//			{
//				double theta = max_theta * M_PI / 180;
//				int r = cols * cos(theta) + rows * sin(theta) + 0.5;
//				if (r < 0 || r >= histo_length)
//					continue;
//
//				if (abs(r - max_r) > 2)
//				{
//					//error++;
//					continue;
//				}
//				PointXY tmp;
//				tmp.x = cols;
//				tmp.y = rows;
//				idx_candidate_new.push_back(tmp);
//				auto t = obsGrid.grids_pt_[idx_grid].points;
//				float mean_z = 0;
//				for (auto iter = t.begin(); iter != t.end(); iter++)
//				{
//					mean_z += iter->z;
//				}
//				mean_z /= t.size();
//				ptz_candidate_new.push_back(mean_z);
//
//			}
//
//		}
//	}
//
//	//for (auto iter = candidate_pts_idx.begin(); iter != candidate_pts_idx.end(); iter++)
//	//{
//	//	int y = *iter / num_width;
//	//	int x = *iter % num_width;
//	//	//float tpx = g_width_unit * (x + g_width_min) + g_center_x + g_width_unit / 2.0;
//	//	//float tpy = g_length_unit * (y + g_length_min) + g_center_y + g_length_unit / 2.0;
//	//	//float thetaR = max_theta / 180.0 * M_PI;
//	//	//float calt_r = x * cos(thetaR) + y * sin(thetaR);
//	//	//if (abs(calt_r - max_r) < 1)
//	//	//{
//	//	PointXY tmp;
//	//	tmp.x = x;
//	//	tmp.y = y;
//	//	idx_candidate.push_back(tmp);
//	//	//}
//	//}
//	//int ptz_size = idx_candidate.size();
//
//	////lcm���ͣ�for debug
//	//lcm::LCM *laserLcm_show_t = new lcm::LCM("udpm://238.255.76.67:7667?ttl=1");
//	//ckLcmType::Draw_t lcmGridShow;
//	//lcmGridShow.timestamp = obsCloud->header.stamp & 0x00000000ffffffffl;
//	////getObstacleGridCloud(Grid_obstacleFlagRes);
//	//for (int i = 0; i < ptz_size; i++)
//	//{
//	//	float ptx = g_width_unit * (idx_candidate[i].x + g_width_min) + g_center_x + g_width_unit / 2.0;
//	//	float pty = g_length_unit * (idx_candidate[i].y + g_length_min) + g_center_y + g_length_unit / 2.0;
//	//	lcmGridShow.x.push_back(ptx);
//	//	lcmGridShow.y.push_back(pty);
//	//}
//	//lcmGridShow.ptnum = ptz_size;
//	//laserLcm_show_t->publish("CKMAPDRAW", &lcmGridShow);
//	//delete laserLcm_show_t;
//	ptz_size = ptz_candidate_new.size();
//	//auto max_z = max_element(ptz_candidate.begin(), ptz_candidate.end());
//
//		for (int i = 0; i < ptz_size; i++)
//		{
//			if (ptz_candidate_new[i] < 0.2)
//			{
//				int idx = idx_candidate_new[i].y*num_width + idx_candidate_new[i].x;
//				for (int j = 0; j < obsGrid.grids_pt_[idx].points_idx.size(); j++)
//				{
//					//	obsIndex.erase(remove(obsIndex.begin(), obsIndex.end(), grids_pt_[idx].points_idx[j]), obsIndex.end());
//					obsGrid.encode_grid_[idx] = 0;
//					obsGrid.grid_obstacle_flag_[idx] = false;
//				}
//			}
//		}
//
//
//	//vector<int> candidate_pts_idx;
//	//
//	//for (int idx_grid = 0; idx_grid < obsGrid.grid_obstacle_flag_.size(); idx_grid++)
//	//{
//	//	if (obsGrid.grid_obstacle_flag_[idx_grid])
//	//	{
//	//		int cols = idx_grid % num_width;
//	//		float ptx = g_width_unit * (cols + g_width_min) + g_center_x + g_width_unit / 2.0;
//	//
//	//		if (ptx > limitD && ptx < limitU)
//	//		{
//	//			candidate_pts_idx.push_back(idx_grid);
//	//		}
//	//
//	//	}
//	//}
//	//
//	//vector<PointXY> point_candidate;
//	//for (int i = 0; i < _grid_obs_points.size(); i++)
//	//{
//	//	if (_grid_obs_points[i].x > limitD&&_grid_obs_points[i].x < limitU)
//	//	{
//	//		PointXY tmp;
//	//		tmp.x = _grid_obs_points[i].x;
//	//		tmp.y = _grid_obs_points[i].y;
//	//		point_candidate.push_back(tmp);
//	//	}
//	//}
//	//int ptz_size = candidate_pts_idx.size();
//	//
//	//
//	//for (int i = 0; i < ptz_size; i++)
//	//{
//	//	int idx = candidate_pts_idx[i];
//	//	//int rows = idx / num_width;
//	//	int cols = idx % num_width;
//	//	float ptx = g_width_unit * (cols + g_width_min) + g_center_x + g_width_unit / 2.0;
//	//	//float pty = g_length_unit * (rows + g_length_min) + g_center_y + g_length_unit / 2.0;
//	//	if (obsGrid.grids_pt_[idx].meanz < 0.25-abs(ptx)*0.025)
//	//	{
//	//		//obsIndex.erase(remove(obsIndex.begin(), obsIndex.end(), obsGrid.grids_pt_[idx].points_idx[j]), obsIndex.end());
//	//		obsGrid.encode_grid_[idx] = 0;
//	//		obsGrid.grid_obstacle_flag_[idx] = false;
//	//	}
//	//}
//
//
//}

void CurbDetection::findNearestObs()
{
	int num = road_fitted_.size();
	int segNum = 10;
	vector<PointXYZI, Eigen::aligned_allocator<PointXYZI> > ptVec_l, ptVec_r;
	//int step = 2;
	for (int i = 0; i < segNum; i++)
	{
		int step = num / segNum;
		int startidx = i* step;
		int endidx = i* step + step - 1;

		PointXYZI pt_l, pt_r;
		dis_movement(startidx, endidx, pt_l, pt_r);
		if (pt_l.x*pt_l.x + pt_l.y*pt_l.y > 0)
			ptVec_l.push_back(pt_l);
		if (pt_r.x*pt_r.x + pt_r.y*pt_r.y > 0)
			ptVec_r.push_back(pt_r);
	}

	PointCloud<PointXYZI> ptVec_fitted_l, ptVec_fitted_r;
	RansacLine line_l, line_r;
	line_l.setPoints(ptVec_l);
	line_r.setPoints(ptVec_r);
	ptVec_fitted_l = line_l.getFittedLine(last_line_para_l);
	ptVec_fitted_r = line_r.getFittedLine(last_line_para_r);
	//pt_vec_left_ = ptVec_l;
	//pt_vec_right_ = ptVec_r;
	line_para_l_ = line_l.line_para_;
	line_para_r_ = line_r.line_para_;

	pt_vec_left_ = line_l.obs_pt_slct_;
	pt_vec_right_ = line_r.obs_pt_slct_;
	if (abs(line_l.line_para_.k_ - line_r.line_para_.k_) < 0.2)
	{
		if (line_l.line_para_.pt_num != 0)
		{
			side_points_left_ = ptVec_fitted_l;
			track_num_l_ = min(track_num_l_ + 1, 10);
		}
		else
		{
			track_num_l_ = max(track_num_l_ - 2, 0);
		}

		if (line_r.line_para_.pt_num != 0)
		{
			//��һ֡�����ݣ��������Ŀ��1
			side_points_right_ = ptVec_fitted_r;
			track_num_r_ = min(track_num_r_ + 1, 10);
		}
		else
		{
			//��һ֡��ʧ��������Ŀ��2
			track_num_r_ = max(track_num_r_ - 2, 0);
		}

		if (track_num_l_ >= 5 && track_num_r_ >= 5)
		{
			if (side_points_left_.empty())
			{
				line_para_l_ = last_line_para_l;
			}
			if (side_points_left_.empty())
			{
				line_para_r_ = last_line_para_r;
			}
			para_way_ = (line_para_l_ + line_para_r_) / 2.0;
		}
		else if (track_num_l_ >= 5)
		{
			if (side_points_left_.empty())
			{
				line_para_l_ = last_line_para_l;
				//para_way_.b_ -= 2.0f;
			}
			para_way_ = line_para_l_;
			para_way_.b_ -= 2.5f;
			//else
			//{
			//	vector<float> ptx_vec;
			//	for (int i = 0; i < side_points_left_.size(); i++)
			//	{
			//		ptx_vec.push_back(side_points_left_[i].x);
			//	}
			//	float mean_move;
			//	if (ptx_vec.size() > 0)
			//		mean_move = accumulate(ptx_vec.begin(), ptx_vec.end(), 0.0) / ptx_vec.size();
			//	else
			//	{
			//		mean_move = 0;
			//	}
			//	para_way_ = line_para_l_;
			//	para_way_.b_ -= max(mean_move, 2.0f);

			//}

		}
		else if (track_num_r_ >= 5)
		{
			if (side_points_right_.empty())
			{
				line_para_r_ = last_line_para_r;
				//para_way_.b_ -= 2.0f;
			}
			para_way_ = line_para_r_;
			para_way_.b_ += 2.5f;
		}
		else
		{

			return;
		}

		//else
		//{
		//	//double step = 40.0 / 100;
		//	//for (int i = 0; i < 100; i++)
		//	//{
		//	//	pcl::PointXYZI tmp;
		//	//	tmp.y = 20 - step * i;
		//	//	pt_vec_way_.push_back(tmp);
		//	//}
		//}

		para_way_ = RansacLine::kalmanFilter(last_para_way, para_way_);

		int p_num = 100;
		float step = 40.0f / p_num;
		for (int i = 0; i < p_num; i++)
		{
			pcl::PointXYZI tmp;

			tmp.y = -20 + i * step;
			tmp.x = para_way_.k_*tmp.y + para_way_.b_;
			tmp.z = -2.0f;
			pt_vec_way_.push_back(tmp);
		}
	}
}


CurbDetection::CurbDetection(const pcl::PointCloud<pcl::PointXYZI>& grid_obs_points)
:_grid_obs_points(grid_obs_points)
{
	float step = 40.0f / 100;
	for (int i = 0; i < 100; i++)
	{
		pcl::PointXYZI tmp;
		tmp.y = 20 - step*i;
		road_fitted_.push_back(tmp);
	}
}

CurbDetection::~CurbDetection()
{

}

void CurbDetection::detectCurb()
{
	findNearestObs();

	CurbDetection::last_para_way = para_way_;
	CurbDetection::last_line_para_l = line_para_l_;
	CurbDetection::last_line_para_r = line_para_r_;
}
