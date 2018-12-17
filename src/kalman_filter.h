//
// Created by howstar on 18-6-5.
//

#ifndef PROJECT_KALMANFILTER_H
#define PROJECT_KALMANFILTER_H

#include <Eigen/Dense>
#include <math.h>

class kalmanFilter {
public:
    kalmanFilter();
    ~kalmanFilter();
public:
    void inputMeasurement_first(float x, float y);
    void inputMeasurement_first(float x, float y, float dx, float dy);

    void inputMeasurement(float x, float y);

    void outputPrediction(float &x, float &y){
        x = statePre(0,0);
        y = statePre(1,0);};

    void outputCorrection(float &x, float &y){
        x = statePost(0,0);
        y = statePost(1,0);};

private:

    void initializeKF();

    void extractorKF();

    void correctMeasure();

    void predictMeasure();

    //variable
public:
    Eigen::MatrixXd measurement;

private:
    Eigen::MatrixXd statePre;           //!< predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k)
    Eigen::MatrixXd statePost;          //!< corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
    Eigen::MatrixXd transitionMatrix;   //!< state transition matrix (A)
    Eigen::MatrixXd controlMatrix;      //!< control matrix (B) (not used if there is no control)
    Eigen::MatrixXd measurementMatrix;  //!< measurement matrix (H)
    Eigen::MatrixXd processNoiseCov;    //!< process noise covariance matrix (Q)
    Eigen::MatrixXd measurementNoiseCov;//!< measurement noise covariance matrix (R)
    Eigen::MatrixXd errorCovPre;        //!< priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)*/
    Eigen::MatrixXd gain;               //!< Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)
    Eigen::MatrixXd errorCovPost;       //!< posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k)

    // temporary matrices
    Eigen::MatrixXd temp1;
    Eigen::MatrixXd temp2;
    Eigen::MatrixXd temp3;
    Eigen::MatrixXd temp4;
    Eigen::MatrixXd temp5;
};


#endif //PROJECT_KALMANFILTER_H
