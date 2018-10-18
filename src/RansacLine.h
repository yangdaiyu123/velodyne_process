#pragma once
#include"stdafx.h"
#include<vector>

using namespace std;

struct VoteInfo
{
	vector<int> match_idx_;
	vector<pcl::PointXYZI> match_pt_;
	vector<double> weigh_pt_;
	double dis_sum_;

	VoteInfo()
	{
		dis_sum_ = 0;
	}
};

class kalmanInfo
{
public:
	float k_;
	float b_;
	float dk_;
	float db_;

	int pt_num;
	//int track_num;

	kalmanInfo()
	{
		k_ = 0;
		b_ = 0;
		dk_ = 0;
		db_ = 0;
		pt_num = 0;
		//track_num = 0;
	}

	const kalmanInfo operator+(const kalmanInfo &r)
	{
		kalmanInfo out;
		out.pt_num = pt_num + r.pt_num;

		out.k_ = (k_*(pt_num + 1) + r.k_*(r.pt_num + 1)) / (out.pt_num + 2) * 2;
		out.b_ = (b_*(pt_num + 1) + r.b_*(r.pt_num + 1)) / (out.pt_num + 2) * 2;

		out.dk_ = sqrt(dk_*dk_ + r.dk_*r.dk_);
		out.db_ = sqrt(db_*db_ + r.db_*r.db_);

		//out.track_num = 0;
		return out;
	}

	friend kalmanInfo operator / (const kalmanInfo &l,const double &r)
	{
		kalmanInfo out;
		out.k_ = l.k_ / r;
		out.b_ = l.b_ / r;
		out.dk_ = l.dk_ / r;
		out.db_ = l.db_ / r;
		out.pt_num = int(l.pt_num / r+0.5);
		//out.track_num = 0;
		return out;
	}
};

class RansacLine
{
public:
	RansacLine();
	~RansacLine();

	void
		setPoints(const vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> >& points);

	pcl::PointCloud<pcl::PointXYZI>
		getFittedLine(kalmanInfo last_para);

	static kalmanInfo kalmanFilter(kalmanInfo para_pre, kalmanInfo para_mea);

private:
	void 
		calt_line_para(kalmanInfo last_para);

//	void
//		calt_curve3_para();

	vector<double> 
		dis_pt2line(kalmanInfo line_para);

private:
	vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> > points_;

	vector<int> pt_idx_select_;
	

public:
	kalmanInfo line_para_;

	pcl::PointXYZI curve_para_;

	vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> > obs_pt_slct_;
	//vector<pcl::PointXYZI, Eigen::aligned_allocator<PointXYZI> > pt_select;
};

