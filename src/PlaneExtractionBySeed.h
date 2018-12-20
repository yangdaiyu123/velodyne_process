#ifndef PLANEEXTRACTIONBYSEED_H_
#define PLANEEXTRACTIONBYSEED_H_

#include "stdafx.h"
#include <stack>
#include <vector>
#include "cloud_show.h"

using pcl::PointXYZI;
using std::pair;
using std::stack;
using std::vector;

class PlaneExtractionBySeed
{
  public:
	PlaneExtractionBySeed(const CloudConstPtr &cloud,
						  const pcl::PointCloud<pcl::PointSrc>::Ptr & cloud_src);
	~PlaneExtractionBySeed();

	bool extractRoadPtsBySeed();

	CloudPtr road() const { return road_; }

  private:
	CloudConstPtr cloud_;
	pcl::PointCloud<pcl::PointSrc>::Ptr cloud_src_;
	int num_sweep_;
	stack<pair<int, int>> st_road_;
	vector<vector<int>> container_;
	vector<int> hazard_pts_;

	void pushStack(int idxFace, int idxBeam);
	void pushStackInit();
	void pushStackRange(int idxFaceLeft, int idxFaceRight, int idxBeam);

	int get_idx(int idxFace, int idxBeam);
	bool isRoadPt(int idxFace, int idxBeam);
	double getDistance(const PointXYZI &p1, const PointXYZI &p2);

	bool isLine(const PointXYZI &p1, const PointXYZI &center, const PointXYZI &p2);
	bool isSingleLineHorizon(int idxSweepLeft, int idxSweepRight, int idxBeam,
							 float &z_max, float &z_min);
	bool isLineHorizon(int idxFaceLeft, int idxFaceRight, int idxBeam);

	Cloud::Ptr road_;
};

#endif // PLANEEXTRACTIONBYSEED_H_