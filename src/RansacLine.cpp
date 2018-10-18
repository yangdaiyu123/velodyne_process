#include "RansacLine.h"

#include<numeric>

using namespace pcl;
using namespace std;

RansacLine::RansacLine()
{
}


RansacLine::~RansacLine()
{
}

//fitLine x=ky+b,return k and b
kalmanInfo
lineFit(vector<pcl::PointXYZI, Eigen::aligned_allocator<PointXYZI> > TestData)
{
	int dataLen = TestData.size();
	Eigen::MatrixXd bMatrix = Eigen::MatrixXd::Ones(dataLen, 2);
	Eigen::MatrixXd lVector = Eigen::VectorXd::Ones(dataLen,1);
	for (int i = 0; i < dataLen; i++)
	{
		bMatrix(i, 0) = TestData[i].y;
		lVector(i) = TestData[i].x;
	}
	Eigen::MatrixXd Qxx = (bMatrix.transpose()*bMatrix).inverse();
	Eigen::MatrixXd para_curb = Qxx*bMatrix.transpose()*lVector;
	Eigen::MatrixXd vVector = bMatrix * para_curb - lVector;
	//float sigma0 = sqrt((vVector.transpose()*vVector)(0) / (dataLen - 2));
	
	kalmanInfo result;
	result.k_ = para_curb(0);
	result.b_ = para_curb(1);
	result.db_ = Qxx(1, 1);
	result.dk_ = Qxx(0, 0);
	result.pt_num = dataLen;
	return result;
}

void Grial(int m, int n, vector<vector<int> > &result)
{
	if (m == n)
	{
		vector<int> t;
		for (int i = 1; i <= m; i++)
		{
			t.push_back(i);
		}
		result.push_back(t);
		return;
	}
	if (n < 2)
	{
		for (int i = 1; i <= m; i++)
		{
			vector<int> t;
			t.push_back(i);
			result.push_back(t);
		}
		return;
	}

	for (int i = m; i >= n; i--)
	{
		vector<vector<int> > tmp_result;
		Grial(i - 1, n - 1, tmp_result);
		for (int m = 0; m < tmp_result.size(); m++)
		{
			vector<int> tmp;
			tmp.push_back(i);
			for (int n = 0; n < tmp_result[m].size(); n++)
			{
				tmp.push_back(tmp_result[m][n]);
			}
			result.push_back(tmp);
		}
	}

}

//��㵽ֱ�ߵľ���
vector<double> 
RansacLine::dis_pt2line(kalmanInfo line_para)
{
	int num = points_.size();
	vector<double> dis_vec(num);
	float k = line_para.k_;
	float b = line_para.b_;
	for (int i = 0; i < num; i++)
	{
		float x = points_[i].y;
		float y = points_[i].x;
		double di = abs(x * k - y + b) / sqrt(1 + k * k);
		dis_vec[i] = di;
	}
	return dis_vec;
}

void RansacLine::setPoints(const vector<pcl::PointXYZI, Eigen::aligned_allocator<PointXYZI> >& points)
{
	points_ = points;
}

pcl::PointCloud<pcl::PointXYZI> RansacLine::getFittedLine(kalmanInfo last_para)
{
	calt_line_para(last_para);
	//calt_curve3_para();
	pcl::PointCloud<pcl::PointXYZI> points_fitted;
	int num = 100;
	float step = 40.0f / num;
	for (int i = 0; i < num; i++)
	{
		pcl::PointXYZI tmp;
		//tmp.y = -20 + i * step;
		////tmp.x = line_para_.x*tmp.y + line_para_.y;
		//tmp.x = curve_para_.x*tmp.y*tmp.y*tmp.y + curve_para_.y*tmp.y*tmp.y + curve_para_.z*tmp.y + curve_para_.intensity;
		//tmp.z = -g_LidarPos[2];
		//points_fitted.push_back(tmp);

		tmp.y = -20 + i * step;
		tmp.x = line_para_.k_*tmp.y + line_para_.b_;
		//tmp.x = curve_para_.x*tmp.y*tmp.y*tmp.y + curve_para_.y*tmp.y*tmp.y + curve_para_.z*tmp.y + curve_para_.intensity;
		tmp.z = -2.0;
		points_fitted.push_back(tmp);
	}
	return points_fitted;
}

kalmanInfo RansacLine::kalmanFilter(kalmanInfo para_pred, kalmanInfo para_mea)
{
	kalmanInfo xk = para_mea;
	xk.db_ = sqrt(para_pred.db_*para_pred.db_ + para_mea.db_*para_mea.db_);
	xk.dk_= sqrt(para_pred.dk_*para_pred.dk_ + para_mea.dk_*para_mea.dk_);
	float kg_b, kg_k;
	if (xk.db_*xk.db_ + para_mea.db_*para_mea.db_ != 0)
	{
		kg_b = (xk.db_*xk.db_) / (xk.db_*xk.db_ + para_mea.db_*para_mea.db_);
		kg_k = (xk.dk_*xk.dk_) / (xk.dk_*xk.dk_ + para_mea.dk_*para_mea.dk_);
	}
	else
	{
		kg_b = 0.f;
		kg_k = 0.f;
	}
	xk.b_ = xk.b_ + kg_b * (para_mea.b_ - xk.b_);
	xk.k_ = xk.k_ + kg_k * (para_mea.k_ - xk.k_);
	xk.db_ = sqrt((1 - kg_b)*xk.db_*xk.db_);
	xk.dk_ = sqrt((1 - kg_k)*xk.dk_*xk.dk_);
	return xk;
}

void RansacLine::calt_line_para(kalmanInfo last_para)
{
	int index1, index2, index3;
	vector<vector<int> > result_idx;
	kalmanInfo result1;
	vector<pcl::PointXYZI, Eigen::aligned_allocator<PointXYZI> > TestData(3);
	int countnum = points_.size();
	if (countnum <= 6)
		return;

	Grial(countnum, 3, result_idx);

	vector<VoteInfo> v;
	int i;
	for (i = 0; i < result_idx.size(); i++)
	{
		index1 = result_idx[i][0];
		index2 = result_idx[i][1];
		index3 = result_idx[i][2];

		TestData[0] = points_[index1-1];
		TestData[1] = points_[index2-1];
		TestData[2] = points_[index3-1];
		result1 = lineFit(TestData);
		vector<double> dis_vec = dis_pt2line(result1);
		int match_size = 0;
		VoteInfo tmp;
		for (int j = 0; j < dis_vec.size(); j++)
		{
			if (dis_vec[j] <= 0.2)
			{
				match_size++;
				tmp.match_idx_.push_back(j);
				tmp.match_pt_.push_back(points_[j]);
				double sqr_dis;
				if (j == 0)
				{
					sqr_dis = (points_[j].x - points_[j + 1].x)*(points_[j].x - points_[j + 1].x) +
						(points_[j].y - points_[j + 1].y)*(points_[j].y - points_[j + 1].y);
				}
				else if (j == points_.size() - 1)
				{
					sqr_dis = (points_[j].x - points_[j - 1].x)*(points_[j].x - points_[j - 1].x) +
						(points_[j].y - points_[j - 1].y)*(points_[j].y - points_[j - 1].y);
				}
				else
				{
					double sqr_dis1, sqr_dis2;
					sqr_dis1 = (points_[j].x - points_[j + 1].x)*(points_[j].x - points_[j + 1].x) +
						(points_[j].y - points_[j + 1].y)*(points_[j].y - points_[j + 1].y);
					sqr_dis2 = (points_[j].x - points_[j - 1].x)*(points_[j].x - points_[j - 1].x) +
						(points_[j].y - points_[j - 1].y)*(points_[j].y - points_[j - 1].y);
					sqr_dis = min(sqr_dis1, sqr_dis2);
				}

				double weigh_pt = sqrt(sqr_dis) / abs((abs(points_[j].x) - 1));
				tmp.weigh_pt_.push_back(weigh_pt);
			}
		}
		if (match_size == dis_vec.size() && match_size > 3)
		{
			break;
		}
		if (match_size < dis_vec.size()*0.6)
			continue;
		
		for (int j = 0; j < match_size; j++)
		{
			tmp.dis_sum_ += dis_vec[tmp.match_idx_[j]];
		}
		v.push_back(tmp);
	}

	if (i < result_idx.size())
	{
		line_para_ = lineFit(points_);

		line_para_ = kalmanFilter(last_para, line_para_);

	}
	else
	{
		if (v.empty()) {
			vector<double> dis_vec = dis_pt2line(last_para);
			int match_size = 0;
			for (auto dis:dis_vec) {
				if (dis < 0.2) match_size++;
			}
			if (match_size > points_.size() * 0.4)
				line_para_ = last_para;
			return;
		}

		vector<double> weight_line;
		for (vector<VoteInfo>::iterator iter = v.begin(); iter != v.end(); iter++)
		{
			int pt_size = iter->match_idx_.size();
			double p_sum = accumulate(iter->weigh_pt_.begin(), iter->weigh_pt_.end(), 0.0);
			weight_line.push_back(p_sum / pt_size * iter->dis_sum_);
			//for (int i = 0; i < pt_size; i++)
			//{
			//	pcl::PointXYZI pt = iter->match_pt_[i];
			//}
		}
		vector<double>::iterator max_iter = max_element(weight_line.begin(), weight_line.end());

		int min_idx = max_iter - weight_line.begin();
		vector<int> t = v[min_idx].match_idx_;

		for (auto idx:t)
		{
			obs_pt_slct_.push_back(points_[idx]);
			pt_idx_select_.push_back(idx);
		}
		line_para_ = lineFit(obs_pt_slct_);


		line_para_ = kalmanFilter(last_para, line_para_);
	}
}


struct vars_struct {
	double *x;
	double *y;
	double *ey;
};

/*
* quadratic fit function
*
* m - number of data points
* n - number of parameters (3)
* p - array of fit parameters
* dy - array of residuals to be returned
* vars - private data (struct vars_struct *)
*
* RETURNS: error code (0 = success)
*/
int quadfunc(int m, int n, double *p, double *dy, double **dvec, void *vars)
{
	int i;
	struct vars_struct *v = (struct vars_struct *) vars;
	double *x, *y, *ey;

	x = v->x;
	y = v->y;
	ey = v->ey;

	/* printf ("quadfunc %f %f %f\n", p[0], p[1], p[2]); */

	for (i = 0; i<m; i++) {
		dy[i] = (y[i] - p[0] - p[1] * x[i] - p[2] * x[i] * x[i]-p[3] * x[i] * x[i] * x[i]) / ey[i];
	}

	return 0;
}

//void RansacLine::calt_curve3_para()
//{
//	/*//int countnum = points_.size();
//	//if (countnum <= 3|| obs_pt_slct_.size()==0)
//	//	return;
//
//	//PointXYZI pt1, pt2;
//	//pt1 = obs_pt_slct_[0];
//	//pt_select.push_back(pt1);
//	//for (int i = pt_idx_select_[0]; i < countnum - 1; i++)
//	//{
//	//	pt1 = points_[i];
//	//	pt2 = points_[i + 1];
//	//	double tmp_k = (pt2.x - pt1.x) / (pt2.y - pt1.y);
//	//	if (abs(tmp_k - line_para_.x) < 0.58)
//	//	{
//	//		pt_select.push_back(pt2);
//	//	}
//	//}
//
//	//int num = pt_select.size();
//	int num = obs_pt_slct_.size();
//	if (num < 4)
//		return;
//
//	Eigen::MatrixXd bMatrix = Eigen::MatrixXd::Ones(num, 4);
//	Eigen::VectorXd lVector = Eigen::VectorXd::Ones(num);
//	//Eigen::Vector4d xVector = Eigen::Vector4d::Zero();
//	for (int i = 0; i < num; i++)
//	{
//		//exchange the x and y to find the mapping from y to x
//		double y = obs_pt_slct_[i].x;
//		double x = obs_pt_slct_[i].y;
//
//		bMatrix(i, 0) = x * x*x;
//		bMatrix(i, 1) = x * x;
//		bMatrix(i, 2) = x;
//
//		lVector(i) = y;
//	}
//
//	Eigen::MatrixXd para_curb = (bMatrix.transpose()*bMatrix).inverse()*bMatrix.transpose()*lVector;
//
//	curve_para_.x = para_curb(0, 0);
//	curve_para_.y = para_curb(1, 0);
//	curve_para_.z = para_curb(2, 0);
//	curve_para_.intensity = para_curb(3, 0);*/
//
//	int num = obs_pt_slct_.size();
//	if (num < 4)
//		return;
//
//	double p[] = { 1.0, 0.0, 1.0 ,1.0};        /* Initial conditions */
//	//double pactual[] = { 4.7, 0.0, 6.2 };  /* Actual values used to make data */
//	double perror[4];		       /* Returned parameter errors */
//	mp_par pars[4];                      /* Parameter constraints */
//	int i;
//	struct vars_struct v;
//	int status;
//	mp_result result;
//
//	memset(&result, 0, sizeof(result));       /* Zero results structure */
//	result.xerror = perror;
//
//	memset(pars, 0, sizeof(pars));       /* Initialize constraint structure */
//	//pars[0].fixed = 1;                   /* Fix parameter 1 */
//	//pars[1].fixed = 1;
//	//pars[2].limited[0] = 0;
//	//pars[2].limits[0] = -0.5;
//	//pars[2].limited[1] = 1;
//	//pars[2].limits[1] = 0.5;
//	//pars[3].limited[0] = 0;
//	//pars[3].limits[0] = -0.1;
//	//pars[3].limited[1] = 1;
//	//pars[3].limits[1] = 0.1;
//
//	double *x = new double[num];
//	double *y = new double[num];
//	double *ey = new double[num];
//	for (i = 0; i < num; i++)
//	{
//		x[i] = obs_pt_slct_[i].y;
//		y[i] = obs_pt_slct_[i].x;
//		ey[i] = 0.2;
//	}
//	v.x = x;
//	v.y = y;
//	v.ey = ey;
//
//	/* Call fitting function for num data points and 4 parameters (2
//	parameter fixed) */
//	status = mpfit(quadfunc, num, 4, p, pars, 0, (void *)&v, &result);
//
//	if (status == 1)
//	{
//		curve_para_.x = p[3];
//		curve_para_.y = p[2];
//		curve_para_.z = p[1];
//		curve_para_.intensity = p[0];
//	}
//
//	delete[] x;
//	delete[] y;
//	delete[] ey;
//}
