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
#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_

#include <opencv2/opencv.hpp>
#include <iostream>

namespace KF_Predict
{
    class KF_Predictor
    {
    public:
        KF_Predictor(int stateNum, int measureNum);
        ~KF_Predictor();
        cv::Point2d KF_Predict(double x, double y, bool start);
        void KF_Init();

    private:
        int stateNum_;   //状态值4×1向量(x,y,△x,△y)
        int measureNum_; //测量值2×1向量(x,y)
        cv::KalmanFilter* KF_;
        cv::Mat measurement_;
    };

}

#endif