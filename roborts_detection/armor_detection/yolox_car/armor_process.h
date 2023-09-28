#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>

template<typename T>
float distance(const cv::Point_<T>& pt1, const cv::Point_<T>& pt2)
{
    return std::sqrt(std::pow((pt1.x - pt2.x), 2) + std::pow((pt1.y - pt2.y), 2));
}

class ArmorProcess{
public:
    ArmorProcess();
    /**
     * @brief Use the sub image to init the class
     * 
     * @param armor_color_ the color of the armor (0 represents BLUE while 1 represents RED)
     * @param img the whole image
     * @param tl the top left position of the sub rectangle
     * @param width the width of the sub rectangle
     * @param height the height of sub rectangle
     */
    void Init(int armor_color, cv::Mat& img, cv::Point2f tl, int width, int height);

    /**
     * @brief find only one couple of matched lights in the image, the light points are stored in lights_position: bl, tl, tr, br
     * 
     * @param lights_position the central positions of the four horizontal lines in two rectangles
     * @return true find the matched lights
     * @return false don't find the matched lighs
     */
    bool FindMatchLights(std::vector<cv::Point2f> &lights_position);

    /**
     * @brief substract the color of the armor
     * 
     * @return cv::Mat gray image
     */
    cv::Mat separateColors();

    /**
     * @brief remove the lights which don't match the pre-conditions
     * 
     * @param lightContours the contours the the lights
     * @param lightInfos the rotated rectangle infos of the lights which match pre-conditions
     */
    void filterContours(std::vector<std::vector<cv::Point> >& lightContours, std::vector<cv::RotatedRect>& lightInfos);

    /**
     * @brief draw lights with rotated rectangles
     * 
     * @param LightInfos the light infos
     */
    void drawLightInfo(std::vector<cv::RotatedRect>& LightInfos);

    /**
     * @brief find only one pair of matched lights which meets pre-conditions
     * 
     * @param lightInfos all the light infos
     * @return true if successfully find one pair of lights which meets pre-conditions
     * @return false 
     */
    bool matchArmor(std::vector<cv::RotatedRect>& lightInfos);

    void adjustRec(cv::RotatedRect& rec);
    
private:
    ros::NodeHandle nh;
    int RED = 1;
    int BLUE = 0;

    //used for light selection
    float light_min_area = 200;
    float light_max_ratio = 0.7;
    float light_contour_min_solidity = 0.3;

    //used for light match
    float light_max_angle_diff_ = 10;
    float light_max_height_diff_ratio_ = 0.2;
    float light_max_y_diff_ratio_ = 0.2;
    float light_min_x_diff_ratio_ = 0.8;
    float light_max_x_diff_ratio_ = 2;
    float armor_min_aspect_ratio_ = 0.3;
    float armor_max_aspect_ratio_ = 4;

    float light_min_dis_ = 30;

    //brightness threshold
    float brightness_threshold_ = 100;

    //debug
    bool enable_debug_ = true;
    bool use_red_enermy = false;

    int armor_color_;
    cv::Rect _roi; //ROI区域
    cv::Point2f tl_;

    cv::Mat _srcImg; //载入的图片保存于该成员变量中
    cv::Mat _roiImg; //从上一帧获得的ROI区域
    cv::Mat _grayImg; //ROI区域的灰度图
    cv::Mat _debugImg;

    ros::NodeHandle *nh_;
};