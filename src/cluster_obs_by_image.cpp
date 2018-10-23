#include "cluster_obs_by_image.h"
#include <pcl/common/time.h>

using namespace cv;
using namespace std;

ObsClusterImg::ObsClusterImg()
    : m_img(m_img_rows, m_img_cols, CV_8U), m_obs_cluster_pts(new pcl::PointCloud<pcl::PointXYZI>)
{
}

ObsClusterImg::~ObsClusterImg()
{
}

void show_image(cv::Mat img, int times, std::string window_name)
{
    Mat img_show;
    resize(img, img_show, img.size() * times);
    imshow(window_name, img_show);
    waitKey(10);
}

void ObsClusterImg::create_img(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                               const std::vector<int> &obs_idx)
{
    m_img = Mat::zeros(m_img.size(), m_img.type());
    m_img.at<uchar>(m_img_rows / 2, m_img_cols / 2) = 128;
    for (unsigned count_pt = 0; count_pt < obs_idx.size(); count_pt++)
    {
        pcl::PointXYZI tmp;
        tmp.x = cloud->points[obs_idx[count_pt]].x;
        tmp.y = cloud->points[obs_idx[count_pt]].y;
        tmp.z = cloud->points[obs_idx[count_pt]].z;
        m_obs_cluster_pts->push_back(tmp);

        int count_col = (int)(m_img_cols / 2 + tmp.x / m_image_res);
        int count_row = (int)(m_img_rows / 2 - tmp.y / m_image_res);

        if (count_col >= 0 && count_col < m_img_cols &&
            count_row >= 0 && count_row < m_img_rows)
        {
            m_img.at<uchar>(count_row, count_col) = 255;
            // int id = count_col + count_row * m_img_cols;

            // grids_pt_[id].points_idx.push_back(obsIndex[count_pt]);
            // grids_pt_[id].points.push_back(p0_t);
        }
    }

    // show_image(m_img,2,"ori");
}

void ObsClusterImg::cluster()
{
    Mat img_dilate;
    int kernal_size = 3;

    cv::Mat element = cv::getStructuringElement(MorphShapes::MORPH_RECT,
                                                cv::Size(kernal_size, kernal_size));

    //膨胀
    cv::dilate(m_img, img_dilate, element);

    // show_image(img_dilate, 2, "dilate res");

    std::vector<std::vector<cv::Point>> contours;
    findContours(img_dilate, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    
    Mat contours_result;
    cvtColor(m_img, contours_result, COLOR_GRAY2BGR);

    drawContours(contours_result, contours, -1, cv::Scalar(0, 0, 255));
    show_image(contours_result, 2, "Contours result");

    // Mat poli_result;
    // cvtColor(m_img,poli_result,COLOR_GRAY2BGR);
    // fillPoly(poli_result,contours,Scalar(255,0,0));
    // show_image(poli_result,2,"polygon");

    for(auto it=contours.begin();it!=contours.end();it++)
    {
        double area=contourArea(*it);

        if(area<40)
        {
            contours.erase(it);
            it--;
            continue;
        }

        Rect rect=boundingRect(*it);
        float ratio=(float)rect.width/(float)rect.height;
        if(ratio<0.2||ratio>5) 
        {
            contours.erase(it);
            it--;
            continue;
        }
    }
    std::cout << "Find " << contours.size() << " Contours" << std::endl;
    std::cout << "obs points num is " << m_obs_cluster_pts->size() << std::endl;

    auto t1=pcl::getTime();
    //TODO:耗时0.8秒左右，太长了
    for (auto &pt : m_obs_cluster_pts->points)
    {
        int count_col = (int)(m_img_cols / 2 + pt.x / m_image_res);
        int count_row = (int)(m_img_rows / 2 - pt.y / m_image_res);

        for (int i=0;i<contours.size();i++)
        {
            auto contour=contours[i];

            if(pointPolygonTest(contour, Point2f(count_col,count_row),false)>0)
            {
                pt.intensity=(i*10)%255;
                break;
            }
        }
    }
    std::cout<<"cluster time is "<<pcl::getTime()-t1<<"s"<<std::endl;
}