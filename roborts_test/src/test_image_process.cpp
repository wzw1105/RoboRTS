#include <ros/ros.h>
#include<iostream>
#include<opencv2/opencv.hpp>
#include<vector>
#include <fstream>
#include <cstdlib>
#include <string>

using namespace std;
using namespace cv;

#define RED 0
#define BLUE 1
#define light_min_area 10
#define light_max_ratio 1
#define light_contour_min_solidity 0.5
#define light_max_angle_diff_ 20
#define light_max_height_diff_ratio_ 0.5
#define light_max_y_diff_ratio_ 0.5
#define light_min_x_diff_ratio_ 0.3
#define armor_min_aspect_ratio_ 0.3
#define armor_max_aspect_ratio_ 2

template<typename T>
float distance(const cv::Point_<T>& pt1, const cv::Point_<T>& pt2)
{
    return std::sqrt(std::pow((pt1.x - pt2.x), 2) + std::pow((pt1.y - pt2.y), 2));
}

class ArmorDetector{
public:
    //初始化各个参数和我方颜色
    void init(int selfColor){
        _self_color = selfColor;
        _enemy_color = (selfColor + 1) % 2;
    }

    void loadImg(Mat& img ){
        _srcImg = img;


        //Rect imgBound = Rect(cv::Point(50, 50), Point(_srcImg.cols - 50, _srcImg.rows- 50) );
        Rect imgBound = Rect(cv::Point(0, 0), Point(_srcImg.cols, _srcImg.rows) );

        _roi = imgBound;
        _roiImg = _srcImg(_roi).clone();//注意一下，对_srcImg进行roi裁剪之后，原点坐标也会移动到裁剪后图片的左上角

    }
    //识别装甲板的主程序，
    int detect(){
        //颜色分离
        std::cout << _srcImg.cols << " " << _srcImg.rows << " " << _srcImg.channels() << "\n\n";
        _grayImg = separateColors();
        imshow("_gray", _grayImg);
        //waitKey(0);
        //std::cout << _grayImg << "\n";


        int brightness_threshold = 90;//设置阈值,取决于你的曝光度
        Mat binBrightImg;
        //阈值化
        std::cout << "before threshold.\n";
        threshold(_grayImg, binBrightImg, brightness_threshold, 255, cv::THRESH_BINARY);
        std::cout << "after threshold\n";
        imshow("thresh", binBrightImg);
        //waitKey(0);

        //膨胀
        Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        dilate(binBrightImg, binBrightImg, element);
        imshow("dilate", binBrightImg);
        //waitKey(0);

        //找轮廓
        vector<vector<Point> > lightContours;
        findContours(binBrightImg.clone(), lightContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        //////debug/////
        
        _debugImg = _roiImg.clone();
        for(size_t i = 0; i < lightContours.size(); i++){
            drawContours(_debugImg,lightContours, i, Scalar(0,0,255),  1, 8);
            double area = contourArea(lightContours[i]);
            cout << "area = " << area << "\n";

        }
        imshow("contours", _debugImg);
        
        //waitKey(0);
        ////////////////

        
        //筛选灯条
        vector<RotatedRect> lightInfos;
        filterContours(lightContours, lightInfos);
        //没找到灯条就返回没找到
        if(lightInfos.empty()){
            return  -1;
        }

        //debug 绘制灯条轮廓
        //drawLightInfo(lightInfos);
        //waitKey(0);
        
        
        //匹配装甲板
        if(!matchArmor(lightInfos)) {
            std::cout << "No armor matched!\n";
            return -1;
        }

        cv::RotatedRect left_light = lightInfos[0], right_light = lightInfos[1];
        if(left_light.center.x > right_light.center.x) std::swap(left_light, right_light);
        std::cout << "left light\n";
        auto vertices = new cv::Point2f[4];
        left_light.points(vertices);
        for(int i = 0; i < 4; i++) std::cout << vertices[i].x << " " << vertices[i].y << "\n";

        drawLightInfo(lightInfos);
    }
    //分离色彩，提取我们需要（也就是敌人）的颜色，返回灰度图
    Mat separateColors(){
        vector<Mat> channels;
        // 把一个3通道图像转换成3个单通道图像
        split(_roiImg,channels);//分离色彩通道

        imshow("split_B", channels[0]);
        imshow("split_G", channels[1]);
        imshow("split_R", channels[2]);
        Mat grayImg;

        //剔除我们不想要的颜色
        //对于图像中红色的物体来说，其rgb分量中r的值最大，g和b在理想情况下应该是0，同理蓝色物体的b分量应该最大,将不想要的颜色减去，剩下的就是我们想要的颜色
        if(_enemy_color==RED){
            grayImg=channels.at(2)-channels.at(0);//R-B
        }
        else{
            grayImg=channels.at(0)-channels.at(2);//B-R
        }
        return grayImg;
    }

    
    //筛选符合条件的轮廓
    //输入存储轮廓的矩阵，返回存储灯条信息的矩阵
    void filterContours(vector<vector<Point> >& lightContours, vector<RotatedRect>& lightInfos){
        int num = lightContours.size();
        for(int i = 0; i < num; i++){
            vector<Point> contour = lightContours[i];
            //RotatedRect contour = lightContours[i]
            //得到面积
            float lightContourArea = contourArea(contour);
            //面积太小的不要
            if(lightContourArea < light_min_area) continue;
            //椭圆拟合区域得到外接矩形
            RotatedRect lightRec = fitEllipse(contour);
            //矫正灯条的角度，将其约束为-45~45
            adjustRec(lightRec);
            //宽高比、凸度筛选灯条  注：凸度=轮廓面积/外接矩形面积
            if(lightRec.size.width / lightRec.size.height > light_max_ratio || lightContourArea / lightRec.size.area() < light_contour_min_solidity)
                continue;
            //对灯条范围适当扩大
            //lightRec.size.width *= _param.light_color_detect_extend_ratio;
            //lightRec.size.height *= _param.light_color_detect_extend_ratio;

            //因为颜色通道相减后己方灯条直接过滤，不需要判断颜色了,可以直接将灯条保存
            lightInfos.push_back(lightRec);
       }
   }


    
    //绘制旋转矩形
    void drawLightInfo(vector<RotatedRect>& LD){
        _debugImg = _roiImg.clone();

        vector<std::vector<cv::Point> > cons;
        int num = LD.size();
        for(int i = 0; i < num; i++){
            RotatedRect rotate = LD[i];
            auto vertices = new cv::Point2f[4];
            rotate.points(vertices);

            vector<Point> con;
            cout << "RotateRect: ";
            for(int i = 0; i < 4; i++) {
                cout << vertices[i] << " ";
                con.push_back(vertices[i]);
            }
            cout << "\n";
            cons.push_back(con);
            drawContours(_debugImg, cons, i, Scalar(0,255,255), 1,8);
            imshow("rotateRectangle", _debugImg);
            //waitKey(0);
        }


    }
    
    //匹配灯条，筛选出装甲板
    bool matchArmor(vector<RotatedRect>& lightInfos){
        int num = lightInfos.size();
        vector<RotatedRect> armors;
       //按灯条中心x从小到大排序
        if(num < 2) return false;
        if(num == 2) return true;

       sort(lightInfos.begin(), lightInfos.end(), [](const RotatedRect& ld1, const RotatedRect& ld2){
           //Lambda函数,作为sort的cmp函数
           return ld1.center.x < ld2.center.x;
       });
       for(size_t i = 0; i < lightInfos.size(); i++){
        //遍历所有灯条进行匹配
           for(size_t j = i + 1; (j < lightInfos.size()); j++){
               const RotatedRect& leftLight  = lightInfos[i];
               const RotatedRect& rightLight = lightInfos[j];

               //角差
               float angleDiff_ = abs(leftLight.angle - rightLight.angle);
               //长度差比率
               float LenDiff_ratio = abs(leftLight.size.height - rightLight.size.height) / max(leftLight.size.height, rightLight.size.height);
               //筛选
               std::cout << i << " " << j << " " << angleDiff_ << " " << LenDiff_ratio << "\n";
               if(angleDiff_ > light_max_angle_diff_ || LenDiff_ratio > light_max_height_diff_ratio_){
                   continue;
               }
               //左右灯条相距距离
               float dis = distance(leftLight.center, rightLight.center);
               //左右灯条长度的平均值
               float meanLen = (leftLight.size.height + rightLight.size.height) / 2;
               //左右灯条中心点y的差值
               float yDiff = abs(leftLight.center.y - rightLight.center.y);
               //y差比率
               float yDiff_ratio = yDiff / meanLen;
               //左右灯条中心点x的差值
               float xDiff = abs(leftLight.center.x - rightLight.center.x);
               //x差比率
               float xDiff_ratio = xDiff / meanLen;
               //相距距离与灯条长度比值
               float ratio = dis / meanLen;
               //筛选
               std::cout << i << " " << j << " " << yDiff_ratio << " " <<  xDiff_ratio << " " << ratio << "\n";
               if(yDiff_ratio > light_max_y_diff_ratio_ || xDiff_ratio < light_min_x_diff_ratio_ || ratio > armor_max_aspect_ratio_ || ratio < armor_min_aspect_ratio_){
                   continue;
               }

               RotatedRect resultLeftLight = leftLight, resultRightLight = rightLight;
               lightInfos.resize(2);
               lightInfos[0] = resultLeftLight, lightInfos[1] = resultRightLight;
               return true;
           }
       }
        return false;
    }

    void adjustRec(cv::RotatedRect& rec)
    {
        using std::swap;

        float& width = rec.size.width;
        float& height = rec.size.height;
        float& angle = rec.angle;
        while(angle >= 90.0) angle -= 180.0;
        while(angle < -90.0) angle += 180.0;


        if(angle >= 45.0)
        {
            swap(width, height);
            angle -= 90.0;
        }
        else if(angle < -45.0)
        {
            swap(width, height);
            angle += 90.0;
        }
    }
    cv::Mat _debugImg;
private:
    int _enemy_color;
    int _self_color;

    cv::Rect _roi; //ROI区域

    cv::Mat _srcImg; //载入的图片保存于该成员变量中
    cv::Mat _roiImg; //从上一帧获得的ROI区域
    cv::Mat _grayImg; //ROI区域的灰度图
    //vector<ArmorDescriptor> _armors;

    //ArmorParam _param;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "image");
    /*Mat img = imread("/home/wangzw/images/red_armor_4.png");
    imshow("_srcImage", img);
    ArmorDetector detector;
    detector.init(BLUE);
    detector.loadImg(img);
    detector.detect();
    //imshow("debug", detector._debugImg);


    waitKey(0);*/
    std::string x;
    ifstream infile;
    infile.open("/home/wangzw/data/train.txt");

    for(int i = 0; i < 10; i++) {
        getline(infile, x);
        cout << string("/home/wangzw/") + x << "\n";
    }
    infile.close();

}

//908 1374