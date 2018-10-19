#ifndef CLUSTER_OBS_BY_IMAGE_H_
#define CLUSTER_OBS_BY_IMAGE_H_

#include <opencv2/opencv.hpp>

#include "stdafx.h"

class ObsClusterImg
{
public:
  ObsClusterImg();
  ~ObsClusterImg();

  void create_img(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                  const std::vector<int> &obs_idx);

  void cluster();

  cv::Mat mat() const { return m_img; };

  pcl::PointCloud<pcl::PointXYZI>::Ptr obs_point() const { return m_obs_cluster_pts; };

private:
  const int m_img_cols = 400;
  const int m_img_rows = 400;
  const float m_image_res = 0.2;

  pcl::PointCloud<pcl::PointXYZI>::Ptr m_obs_cluster_pts;

  cv::Mat m_img;
};

#endif