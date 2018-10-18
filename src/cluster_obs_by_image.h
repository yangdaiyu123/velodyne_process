#ifndef CLUSTER_OBS_BY_IMAGE_H_
#define CLUSTER_OBS_BY_IMAGE_H_

#include <opencv2/opencv.hpp>

#include "stdafx.h"

class ObsClusterImg
{
  public:
    ObsClusterImg();
    ~ObsClusterImg();

    void create_img(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                    const std::vector<int> &obs_idx);

    void cluster();

    cv::Mat mat() const { return m_img; };

  private:
    const int m_img_cols = 150;
    const int m_img_rows = 400;
    const float m_image_res = 0.2;

    cv::Mat m_img;
};

#endif