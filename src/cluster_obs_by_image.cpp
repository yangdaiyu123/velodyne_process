#include "cluster_obs_by_image.h"

using namespace cv;
using namespace std;

ObsClusterImg::ObsClusterImg()
    :m_img(m_img_rows,m_img_cols,CV_8U)
{
}

ObsClusterImg::~ObsClusterImg()
{
}

void ObsClusterImg::create_img(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                               const std::vector<int> &obs_idx)
{
    for (unsigned count_pt = 0; count_pt < obs_idx.size(); count_pt++)
    {
        float x = cloud->points[obs_idx[count_pt]].x;
        float y = cloud->points[obs_idx[count_pt]].y;
        float z = cloud->points[obs_idx[count_pt]].z;

        pcl::PointXYZI temp1;
        temp1.x = x;
        temp1.y = y;
        temp1.z = z;
        pcl::PointXYZI p0_t = transport_point(temp1);
        //        pcl::PointXYZI p0_t= temp1;
        x = p0_t.x;
        y = p0_t.y;
        z = p0_t.z;
        

        int count_width = (int)(m_img_cols / 2 + x / m_image_res);
        int count_row = (int)(m_img_rows / 2 - y / m_image_res);

        if (count_width >= 0 && count_width < m_img_cols &&
            count_row >= 0 && count_row < m_img_rows)
        {
            m_img.at<uchar>(count_width,count_row)=255;
            // int id = count_width + count_row * m_img_cols;

            // grids_pt_[id].points_idx.push_back(obsIndex[count_pt]);
            // grids_pt_[id].points.push_back(p0_t);
        }
    }
    
    imshow("res",m_img);
    waitKey(0);
}