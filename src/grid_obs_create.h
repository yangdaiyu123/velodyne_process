#ifndef GRID_OBS_CREATE_H_
#define GRID_OBS_CREATE_H_
#include"stdafx.h"

struct GridInfo
{
	std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> > points;
	std::vector<int> points_idx;
	float meanz;
	float stdz;

	GridInfo()
	{
		meanz = 0;
		stdz = 0;
	}
};


class GridCreator
{
public:
	GridCreator();
	~GridCreator();

	void createGrid(pcl::PointCloud<pcl::PointXYZI>::ConstPtr obsCloud, 
					std::vector<int>& obsIndex);

private:

	void calGridInfo(pcl::PointCloud<pcl::PointXYZI>::ConstPtr obsCloud, 
					std::vector<int>& obsIndex);

	int m_grid_size;

public:

	std::vector<int8_t> encode_grid_;

	std::vector<GridInfo> grids_pt_;

    pcl::PointCloud<pcl::PointXYZI> grid_obs_points_;

	int m_num_width;
	int m_num_length;

};

#endif