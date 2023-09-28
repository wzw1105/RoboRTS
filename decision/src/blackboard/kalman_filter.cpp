/*
 * @content: 保持安静
 * @Description: file 保持安静
 * @author: Chen
 */
/*
 * @content: 保持安静
 * @Description: file 保持安静
 * @author: Chen
 */
/*
 * @content: 保持安静
 * @Description: file 保持安静u
 * @author: Chen
 */

#include "kalman_filter.h"

namespace KF_Predict
{
    KF_Predictor::KF_Predictor(int stateNum, int measureNum)
        {
            stateNum_ = stateNum;
            measureNum_ = measureNum;
            measurement_ = (cv::Mat_<float>(2, 1) << 0, 0);
            KF_ = new cv::KalmanFilter(stateNum_, measureNum, 0);
        }                                    //后验错误估计协方差矩阵
   

    KF_Predictor::~KF_Predictor(){
        delete KF_;
    }

    void KF_Predictor::KF_Init()
    {
        KF_->transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1); //转移矩阵A
        setIdentity(KF_->measurementMatrix);                                                               //测量矩阵H
        setIdentity(KF_->processNoiseCov, cv::Scalar::all(1e-5));                                          //过程噪声方差矩阵Q
        setIdentity(KF_->measurementNoiseCov, cv::Scalar::all(1e-1));                                      //测量噪声方差矩阵R
        setIdentity(KF_->errorCovPost, cv::Scalar::all(1)); 
    }
    cv::Point2d KF_Predictor::KF_Predict(double x, double y, bool start)
    {
        cv::Point2d predict_point(-1, -1);
        if (start)
        {
            KF_Init();
            KF_->statePost = (cv::Mat_<double>(4, 1) << 0, 0, 0, 0);
            measurement_.at<double>(0, 0) = x;
            measurement_.at<double>(1, 0) = y;
            KF_->correct(measurement_);
        }
        else
        {
            //* 更新
            measurement_.at<double>(0) = x;
            measurement_.at<double>(1) = y;
            KF_->correct(measurement_);
            //* 获取预测值
            cv::Mat predict_location = KF_->predict();
            predict_point = cv::Point(predict_location.at<double>(0), predict_location.at<double>(1));
        }
        return predict_point;
    }
}
