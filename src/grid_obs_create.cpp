#include "grid_obs_create.h"

#define OBSSELF_DITANCE_M 1.7

using namespace std;

GridCreator::GridCreator()
	: encode_grid_(0), m_num_length(400), m_num_width(150), m_grid_size(0.2)
{
}

GridCreator::~GridCreator()
{
}

//void
//GridCreator::obs_points_extract()
//{
//	int grid_width = 75 - g_width_min;
//	int grid_height = 200 - g_length_min;
//	int num_grid = grid_width*grid_height;
//	//char path[100];
//	//sprintf(path, "../../data_analyze/allData/obs%d.txt", g_index);
//	//ofstream of(path);
//	for (int idx = 0; idx < num_grid; idx++)
//	{
//		if (encode_grid_[idx] > 0)
//		{
//			int i = idx % grid_width;
//			int j = idx / grid_width;
//			pcl::PointXYZI tmp;
//			tmp.x = (i + g_width_min)*0.2 + 0.2 / 2;
//			tmp.y = (j + g_length_min)*0.2 + 0.2 / 2;
//			tmp.z = 0;
//			tmp.intensity = 0;
//
//			//of << tmp.x << " " << tmp.y << " " << tmp.z << endl;
//			_grid_obs_points->points.push_back(tmp);
//		}
//	}
//	//of.close();
//}

void GridCreator::createGrid(pcl::PointCloud<pcl::PointXYZI>::ConstPtr obsCloud,
							 vector<int> &obsIndex)
{
	encode_grid_.resize(m_num_width * m_num_length, 0);
	grids_pt_.resize(m_num_width * m_num_length, GridInfo());

	for (unsigned count_pt = 0; count_pt < obsIndex.size(); count_pt++)
	{
		float x = obsCloud->points[obsIndex[count_pt]].x;
		float y = obsCloud->points[obsIndex[count_pt]].y;
		float z = obsCloud->points[obsIndex[count_pt]].z;

		pcl::PointXYZI temp1;
		temp1.x = x;
		temp1.y = y;
		temp1.z = z;
		pcl::PointXYZI p0_t = transport_point(temp1);
		//        pcl::PointXYZI p0_t= temp1;
		x = p0_t.x;
		y = p0_t.y;
		z = p0_t.z;
		int count_width, count_length;

		count_width = (int)(m_num_width / 2 + x / m_grid_size);
		count_length = (int)(m_num_length - y / m_grid_size);

		if (count_width >= 0 && count_width < m_num_width &&
			count_length >= 0 && count_length < m_num_length)
		{
			int id = count_width + count_length * m_num_width;

			grids_pt_[id].points_idx.push_back(obsIndex[count_pt]);
			grids_pt_[id].points.push_back(p0_t);
		}
	}

	calGridInfo(obsCloud, obsIndex);
}

void GridCreator::calGridInfo(pcl::PointCloud<pcl::PointXYZI>::ConstPtr obsCloud,
							  vector<int> &obsIndex)
{
	for (int i = 0; i < grids_pt_.size(); i++)
	{
		double mean = 0;
		double stdev = 0;
		int pnum = grids_pt_[i].points.size();
		if (pnum > 1)
		{
			int rows = i / m_num_width;
			int cols = i % m_num_width;
			float ptx = m_grid_size * (cols - m_num_width / 2) + m_grid_size / 2.0;
			float pty = m_grid_size * (rows - m_num_length / 2) + m_grid_size / 2.0;

			for (int j = 0; j < pnum; j++)
				mean += grids_pt_[i].points[j].z;

			mean /= pnum;
			grids_pt_[i].meanz = mean;

			for (int j = 0; j < pnum; j++)
				stdev += (grids_pt_[i].points[j].z - mean) * (grids_pt_[i].points[j].z - mean);

			stdev = sqrt(stdev / (pnum - 1));
			grids_pt_[i].stdz = stdev;

			if (mean > 0)
			{
				encode_grid_[i] = 1;
			}
			else
			{
				encode_grid_[i] = 2;
			}
		}
	}
}
