#ifndef VELOPROCESS_SRC_OBSDETECTION_H_
#define VELOPROCESS_SRC_OBSDETECTION_H_

#include "stdafx.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

//新注册点类型，给点云赋予索引行和列
//行列分别是指它所在格网的行和列
struct PointXYZIndex
{
  PCL_ADD_POINT4D;
  // union {
  //   float coordinate[3];
  //   struct
  //   {
  //     float x;
  //     float y;
  //     float z;
  //   };
  // };
  int row;
  int col;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIndex, // 注册点类型宏
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (int, row, row)
                                  (int, col, col)
                                  )

class ObstacleDetection
{
public:
  ObstacleDetection();
  ~ObstacleDetection();

  void detectObstacle(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, std::vector<int>& obs_idx);

  void transformXYZToIndex(PointXYZIndex& pt);

private:
  pcl::PointCloud<PointXYZIndex>::Ptr cloud_road_;

  const int grid_width_num_ = 300;
  const int grid_length_num_ = 600;
  const float grid_resolution_ = 0.2;

  //private function
  void growRoad();


};

#endif