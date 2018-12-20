#include "PlaneExtractionBySeed.h"

PlaneExtractionBySeed::PlaneExtractionBySeed(const CloudConstPtr &cloud,
											 const pcl::PointCloud<pcl::PointSrc>::Ptr &cloud_src)
	: cloud_(cloud),
	  cloud_src_(cloud_src),
	  road_(new Cloud),
	  num_sweep_(cloud_->size() / kLidarLine),
	  container_(num_sweep_, vector<int>(kLidarLine, 0))
{
	//for (int i = 0; i < hazard_pts_.size(); i++)
	//{
	//	int idx = hazard_pts_[i];
	//	int idxBeam = idx % 32;
	//	int idxSweep = idx / 32;
	//	idxBeam= (idxBeam % 2 == 0) ? (idxBeam / 2) : (idxBeam / 2 + 16);
	//	container_[idxSweep][idxBeam] = 1;
	//}
}

PlaneExtractionBySeed::~PlaneExtractionBySeed()
{
}

bool PlaneExtractionBySeed::extractRoadPtsBySeed()
{
	pushStackInit();

	while (!st_road_.empty())
	{
		pair<int, int> pt_idx = st_road_.top();
		st_road_.pop();

		isRoadPt(pt_idx.first, pt_idx.second);
	}

	return true;
}

int PlaneExtractionBySeed::get_idx(int idx_sweep, int idx_beam)
{
	int idx = idx_beam < 8 ? idx_beam : 23 - idx_beam;
	return (unsigned)(idx * num_sweep_ + idx_sweep);
}

bool PlaneExtractionBySeed::isRoadPt(int idxSweep, int idxBeam)
{
	int range_points = 20;
	int idxSweepLeft = (idxSweep - range_points + num_sweep_) % num_sweep_; //超限之后从头开始计数
	int idxSweepRight = (idxSweep + range_points) % num_sweep_;				//超限之后从头开始计数
	int idxSweepFront = std::min(kLidarLine - 1, idxBeam + 1);
	int idxSweepBehind = std::max(0, idxBeam - 1);

	PointXYZI seed = cloud_->at(get_idx(idxSweep, idxBeam));
	PointXYZI seed_l = cloud_->at(get_idx(idxSweepLeft, idxBeam));
	PointXYZI seed_r = cloud_->at(get_idx(idxSweepRight, idxBeam));
	PointXYZI seed_f = cloud_->at(get_idx(idxSweep, idxSweepFront));
	PointXYZI seed_b = cloud_->at(get_idx(idxSweep, idxSweepBehind));

	if (isLineHorizon(idxSweepLeft, idxSweepRight, idxBeam))
	{
		pushStackRange(idxSweepLeft, idxSweepRight, idxBeam);
		//pushStack(idxSweep, idxBeam);
		//pushStack(idxSweepLeft, idxBeam);
		//pushStack(idxSweepRight, idxBeam);
	}
	if (isLine(seed_b, seed, seed_f))
	{
		pushStack(idxSweep, idxBeam);
		pushStack(idxSweep, idxSweepBehind);
		pushStack(idxSweep, idxSweepFront);

		// //for bebug
		// sensor_msgs::PointCloud2 output2;
		// pcl::toROSMsg(*road_, output2);
		// output2.header.frame_id = "frame_id";
		// cloud_show::pub_obs.publish(output2);
		// usleep(100);
	}

	return true;
}

double PlaneExtractionBySeed::getDistance(const PointXYZI &p1, const PointXYZI &p2)
{
	double dx, dy, dz;
	dx = p2.x - p1.x;
	dy = p2.y - p1.y;
	dz = p2.z - p1.z;
	return sqrt(dx * dx + dy * dy + dz * dz);
}

bool PlaneExtractionBySeed::isLine(const PointXYZI &p1, const PointXYZI &center, const PointXYZI &p2)
{
	PointXYZI v1, v2;
	v1.x = p1.x - center.x;
	v1.y = p1.y - center.y;
	v1.z = p1.z - center.z;
	v2.x = p2.x - center.x;
	v2.y = p2.y - center.y;
	v2.z = p2.z - center.z;

	double d1 = getDistance(p1, center);
	double d2 = getDistance(p2, center);
	if (d1 == 0 || d2 == 0)
	{
		return false;
	}

	double angle = acos((v1.x * v2.x + v1.y * v2.y + v1.z * v2.z) / d1 / d2);
	double d = std::max(d1, d2);
	double angle_threshold = atan(0.1 / d); //�����ϰ���߶�Ϊ0.1��
	return abs(angle - M_PI) < angle_threshold;
}

bool PlaneExtractionBySeed::isSingleLineHorizon(int idxSweepLeft, int idxSweepRight, int idxBeam,
												float &z_max, float &z_min)
{
	// if (idxSweepRight - idxSweepLeft < 7)
	// {
	int idx_mean = (idxSweepRight + idxSweepLeft) / 2;

	z_min = 100;
	z_max = -100;
	for (int i = idxSweepLeft; i < idxSweepRight; i++)
	{
		auto tmp = cloud_->at(get_idx(i, idxBeam));
		auto tmp1 = cloud_->at(get_idx(i + 1, idxBeam));
		auto tmp_src = cloud_src_->at(get_idx(i, idxBeam));
		auto tmp_src1 = cloud_src_->at(get_idx(i + 1, idxBeam));
		if (tmp.z < z_min)
			z_min = tmp.z;
		if (tmp.z > z_max)
			z_max = tmp.z;

		float d = abs(tmp_src.radius - tmp_src1.radius);
		if (d > 0.1)
		{
			return false;
		}
	}
	// }
	// else
	// {
	// 	int pt_size = idxSweepRight - idxSweepLeft + 1;
	// 	Eigen::MatrixXd bMatrix = Eigen::MatrixXd::Ones(pt_size, 5);
	// 	Eigen::MatrixXd lVector = Eigen::VectorXd::Ones(pt_size, 1);
	// 	for (int i = idxSweepLeft; i < idxSweepRight; i++)
	// 	{
	// 		int idx = get_idx(i, idxBeam);
	// 		auto pt = cloud_->at(idx);
	// 		int ii=i-idxSweepLeft;
	// 		bMatrix(ii, 0) = pt.x * pt.x;
	// 		bMatrix(ii, 1) = pt.x * pt.y;
	// 		bMatrix(ii, 2) = pt.y * pt.y;
	// 		bMatrix(ii, 3) = pt.x;
	// 		lVector(ii) = pt.y;
	// 	}
	// 	Eigen::MatrixXd Qxx = (bMatrix.transpose() * bMatrix).inverse();
	// 	Eigen::MatrixXd parameter_ellipse = Qxx * bMatrix.transpose() * lVector;
	// 	Eigen::MatrixXd vVector = bMatrix * parameter_ellipse - lVector;

	// 	if(vVector.norm()>0.01)
	// 	{
	// 		return false;
	// 	}
	// }
	return true;
}

bool PlaneExtractionBySeed::isLineHorizon(int idxSweepLeft, int idxSweepRight, int idxBeam)
{
	float z_max = -100;
	float z_min = 100;
	if (idxSweepLeft > idxSweepRight)
	{
		if (!isSingleLineHorizon(idxSweepLeft, num_sweep_ - 1, idxBeam, z_max, z_min))
		{
			return false;
		}
		if (!isSingleLineHorizon(0, idxSweepRight, idxBeam, z_max, z_min))
		{
			return false;
		}
	}
	else
	{
		if (!isSingleLineHorizon(idxSweepLeft, idxSweepRight, idxBeam, z_max, z_min))
		{
			return false;
		}
	}

	if (z_max - z_min > 0.1)
	{
		return false;
	}

	return true;
}

void PlaneExtractionBySeed::pushStack(int idxSweep, int idxBeam)
{
	if (container_[idxSweep][idxBeam] == 0)
	{
		int idx = get_idx(idxSweep, idxBeam);
		auto seed = cloud_->at(idx);

		road_->push_back(seed);

		st_road_.push(pair<int, int>(idxSweep, idxBeam));
		container_[idxSweep][idxBeam] = 1;

		// //for bebug
		// sensor_msgs::PointCloud2 output2;
		// pcl::toROSMsg(*road_, output2);
		// output2.header.frame_id = "rslidar";
		// cloud_show::pub_obs.publish(output2);
		// usleep(10000);
	}
}

void PlaneExtractionBySeed::pushStackInit()
{
	for (int i = 0; i < cloud_->size(); i++)
	{
		auto one = cloud_->points[i];
		if (abs(one.z) < 0.01)
		{
			int idx_sweep = i % num_sweep_;
			int idx_beam = i / num_sweep_;
			idx_beam = idx_beam < 8 ? idx_beam : 23 - idx_beam;
			pushStack(idx_sweep, idx_beam);
		}
	}
	// st_road_.push(pair<int, int>(idxSweep, idxBeam));
	// st_road_.push(pair<int, int>(idxSweep, std::max(0, idxBeam - 1)));
	// st_road_.push(pair<int, int>(idxSweep, std::min(31, idxBeam + 1)));
	// st_road_.push(pair<int, int>((idxSweep + 1) % num_sweep_, idxBeam));
	// st_road_.push(pair<int, int>((idxSweep - 1 + num_sweep_) % num_sweep_, idxBeam));
}

void PlaneExtractionBySeed::pushStackRange(int idxSweepLeft, int idxSweepRight, int idxBeam)
{
	if (idxSweepLeft < idxSweepRight)
	{
		for (int i = idxSweepLeft; i < idxSweepRight; i++)
		{
			pushStack(i, idxBeam);
		}
	}
	else
	{
		for (int i = idxSweepLeft; i < num_sweep_; i++)
		{
			pushStack(i, idxBeam);
		}
		for (int i = 0; i < idxSweepRight; i++)
		{
			pushStack(i, idxBeam);
		}
	}
}
