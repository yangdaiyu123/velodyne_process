//
// Created by howstar on 18-6-5.
//

#include "kalmanFilter.h"

kalmanFilter::kalmanFilter()
{
    initializeKF();
}

kalmanFilter::~kalmanFilter()
{

}

void kalmanFilter::initializeKF()
{
    int DP = 4;
    int MP = 2;
    int CP = 0;

    //CP = std::max(1.0 * CP, 0.0);
    measurement = Eigen::MatrixXd(MP,1);

    statePre = Eigen::MatrixXd(DP,1);

    statePost = Eigen::MatrixXd(DP,1);

    transitionMatrix = Eigen::MatrixXd(DP,DP);
    //std::cout<<transitionMatrix<<std::endl;

    processNoiseCov = Eigen::MatrixXd::Identity(DP, DP);
    /*for (int i=0;i<DP;i++)
    {
    processNoiseCov(i,i) = 1.0;
    }*/

    measurementMatrix = Eigen::MatrixXd(MP,DP);

    //噪声协因数矩阵：单位阵
    measurementNoiseCov = Eigen::MatrixXd::Identity(MP,MP);
    /*for (int i=0;i<MP;i++)
    {
    measurementNoiseCov.at<float>(i,i) = 1.0;
    }*/

    errorCovPre = Eigen::MatrixXd(DP,DP);

    errorCovPost = Eigen::MatrixXd(DP,DP);

    gain = Eigen::MatrixXd(DP,MP);

    if( CP > 0 )
        controlMatrix = Eigen::MatrixXd(DP, CP);
    else
        //controlMatrix.release();

        temp1 = Eigen::MatrixXd(DP, DP);
    temp2 = Eigen::MatrixXd(MP, DP);
    temp3 = Eigen::MatrixXd(MP, MP);
    temp4 = Eigen::MatrixXd(MP, DP);
    temp5 = Eigen::MatrixXd(MP, 1 );

    //evaluate
    transitionMatrix<<1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1;
    measurementMatrix.setIdentity();
    processNoiseCov.setIdentity();
    processNoiseCov = 5e-3 * processNoiseCov;//control smooth
    measurementNoiseCov.setIdentity();
    measurementNoiseCov = 0.5 * measurementNoiseCov;
    errorCovPost.setIdentity();
    errorCovPost = 0.5 * errorCovPost;
}

void kalmanFilter::inputMeasurement_first(float x, float y)
{
    statePre(0,0) = x;
    statePre(1,0) = y;
    statePre(2,0) = 0.0;
    statePre(3,0) = 0.0;

    statePost(0,0) = x;
    statePost(1,0) = y;
    statePost(2,0) = 0.0;
    statePost(3,0) = 0.0;

    measurement(0,0) = x;
    measurement(1,0) = y;

    predictMeasure();
}

void kalmanFilter::inputMeasurement_first(float x, float y, float dx, float dy)
{
    statePre(0,0) = x;
    statePre(1,0) = y;
    statePre(2,0) = dx;
    statePre(3,0) = dy;

    statePost(0,0) = x;
    statePost(1,0) = y;
    statePost(2,0) = dx;
    statePost(3,0) = dy;

    measurement(0,0) = x;
    measurement(1,0) = y;

    predictMeasure();
}

void kalmanFilter::inputMeasurement(float x, float y)
{
    measurement(0,0) = x;
    measurement(1,0) = y;
    correctMeasure();
    predictMeasure();
}

void kalmanFilter::predictMeasure()
{
    //const Eigen::MatrixXd & control;
    ///*1.predicted state estimate X(k|k-1) = F * X(k|k-1) + B(k) * u
    // update the state: x'(k) = A*x(k)
    statePre = transitionMatrix*statePost;
    //std::cout<<transitionMatrix<<std::endl;

    //if( control.data )
    //// x'(k) = x'(k) + B*u(k)
    //statePre += controlMatrix*control;

    ///*2.predicted estimate covariance P(k|k-1) = F * P(k-1|k-1) * F.t() + Q
    // update error covariance matrices: temp1 = A*P(k)
    temp1 = transitionMatrix*errorCovPost;

    // P'(k) = temp1*At + Q
    //gemm(temp1, transitionMatrix, 1, processNoiseCov, 1, errorCovPre, GEMM_2_T);
    errorCovPre = temp1 * transitionMatrix.transpose() + processNoiseCov;

    // handle the case when there will be measurement before the next predict.
    //statePre.copyTo(statePost);
    //errorCovPre.copyTo(errorCovPost);
    //statePost = statePre;
    //errorCovPost = errorCovPre;

    //for test//std::cout<<"state pos:"<<statePre(0,0)<<"	"<<statePre(1,0)<<"	"<<statePre(2,0)<<"	"<<statePre(3,0)<<std::endl;

    //return statePre;
}

void kalmanFilter::correctMeasure()
{
    ///*4.innovation or residual covariance S(k) = H(k) * P(k|k-1) * H(k).t() + R
    // temp2 = H*P'(k)
    temp2 = measurementMatrix * errorCovPre;

    // temp3 = temp2*Ht + R
    //gemm(temp2, measurementMatrix, 1, measurementNoiseCov, 1, temp3, GEMM_2_T);
    temp3 = temp2 * measurementMatrix.transpose() + measurementNoiseCov;

    ///*5.optimal Kalman Filter K(k) = P(k|k-1) * H(k).t() * S(k).inv()
    // temp4 = inv(temp3)*temp2 = Kt(k)
    //solve(temp3, temp2, temp4, DECOMP_SVD);
    temp4 = temp3.colPivHouseholderQr().solve(temp2);

    // K(k)
    gain = temp4.transpose();

    ///*3.innovation or measurement residual y(k) = z(k) - H(k) * x(k|k-1)
    // temp5 = z(k) - H*x'(k)
    temp5 = measurement - measurementMatrix*statePre;

    // x(k) = x'(k) + K(k)*temp5
    statePost = statePre + gain*temp5;

    // P(k) = P'(k) - K(k)*temp2
    errorCovPost = errorCovPre - gain*temp2;

    //return statePost;
}