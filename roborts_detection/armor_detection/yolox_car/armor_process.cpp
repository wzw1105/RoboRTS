#include "armor_process.h"

ArmorProcess::ArmorProcess() {

    //used for light selection
    nh.param<float>("light_min_area", light_min_area, 200);
    nh.param<float>("light_max_ratio", light_max_ratio, 0.7);
    nh.param<float>("light_contour_min_solidity", light_contour_min_solidity, 0.3);

    //used for light match
    nh.param<float>("light_max_angle_diff_", light_max_angle_diff_, 10);
    nh.param<float>("light_max_height_diff_ratio_", light_max_height_diff_ratio_, 0.2);
    nh.param<float>("light_max_y_diff_ratio_", light_max_y_diff_ratio_, 0.2);
    nh.param<float>("light_min_x_diff_ratio_", light_min_x_diff_ratio_, 0.8);
    nh.param<float>("light_max_x_diff_ratio_", light_max_x_diff_ratio_, 2);
    nh.param<float>("armor_min_aspect_ratio_", armor_min_aspect_ratio_, 0.3);
    nh.param<float>("armor_max_aspect_ratio_", armor_max_aspect_ratio_, 4);
    nh.param<float>("light_min_dis_", light_min_dis_, 30);
    nh.param<bool>("use_red_enermy", use_red_enermy, false);

    //brightness threshold
    nh.param<float>("brightness_threshold_", brightness_threshold_, 100);
    nh.param<bool>("enable_debug_", enable_debug_, false);    
}


void ArmorProcess::Init(int armor_color, cv::Mat& img, cv::Point2f tl, int width, int height){

    armor_color_ = armor_color;
    _srcImg = img;
    tl_ = tl;
    cv::Rect imgBound = cv::Rect(cv::Point(std::max(0.0f, tl.x), std::max(0.0f, tl.y)), cv::Point(std::min((float)(_srcImg.cols - 1), tl.x + width), std::min(float(_srcImg.rows - 1), tl.y + height)));
    _roi = imgBound;
    _roiImg = _srcImg(_roi).clone();//注意一下，对_srcImg进行roi裁剪之后，原点坐标也会移动到裁剪后图片的左上角
}

bool ArmorProcess::FindMatchLights(std::vector<cv::Point2f> &lights_position) {
    //颜色分离
    _grayImg = separateColors();
    if(enable_debug_) cv::imshow("_gray", _grayImg);

    //medianBlur(_grayImg, _grayImg, 5);
    //cv::GaussianBlur(_grayImg, _grayImg, cv::Size(5, 5), 0);
    //cv::blur(_grayImg, _grayImg, cv::Size(7, 7));
    //if(enable_debug_) imshow("blur", _grayImg);

    auto detection_begin = std::chrono::high_resolution_clock::now();

    //阈值化
    int brightness_threshold = brightness_threshold_;//设置阈值,取决于你的曝光度
    cv::Mat binBrightImg;
    threshold(_grayImg, binBrightImg, brightness_threshold, 255, cv::THRESH_BINARY);
    if(enable_debug_) imshow("thresh", binBrightImg);

    if(enable_debug_) {
        ROS_WARN("thresh: %.4lf", std::chrono::duration<double, std::ratio<1, 1000000> >(std::chrono::high_resolution_clock::now() - detection_begin).count());
        detection_begin = std::chrono::high_resolution_clock::now();
    }

    //膨胀
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    dilate(binBrightImg, binBrightImg, element);
    if(enable_debug_) imshow("dilate", binBrightImg);
    if(enable_debug_) {
        ROS_WARN("diltate: %.4lf", std::chrono::duration<double, std::ratio<1, 1000000> >(std::chrono::high_resolution_clock::now() - detection_begin).count());
        detection_begin = std::chrono::high_resolution_clock::now();
    }
    //找轮廓
    std::vector<std::vector<cv::Point> > lightContours;
    findContours(binBrightImg.clone(), lightContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    if(enable_debug_) {
        ROS_WARN("find contours: %.4lf", std::chrono::duration<double, std::ratio<1, 1000000> >(std::chrono::high_resolution_clock::now() - detection_begin).count());
        detection_begin = std::chrono::high_resolution_clock::now();
    }
    //debug
    if(enable_debug_) {
        _debugImg = _roiImg.clone();
        for(size_t i = 0; i < lightContours.size(); i++){
            drawContours(_debugImg,lightContours, i, cv::Scalar(0,0,255),  1, 8);
        }
        imshow("contours", _debugImg);
    }
    
    //筛选灯条
    std::vector<cv::RotatedRect> lightInfos;
    filterContours(lightContours, lightInfos);
    if(lightInfos.empty()) return false;


    if(enable_debug_) {
        ROS_WARN("filter contours: %.4lf", std::chrono::duration<double, std::ratio<1, 1000000> >(std::chrono::high_resolution_clock::now() - detection_begin).count());
        detection_begin = std::chrono::high_resolution_clock::now();
    }

    std::sort(lightInfos.begin(), lightInfos.end(), [](const cv::RotatedRect &p1, const cv::RotatedRect &p2) {
        return p1.size.height > p2.size.height;
    });
    
    //匹配装甲板
    if(!matchArmor(lightInfos)) return false;


    if(enable_debug_) {
        ROS_WARN("find contours: %.4lf", std::chrono::duration<double, std::ratio<1, 1000000> >(std::chrono::high_resolution_clock::now() - detection_begin).count());
        detection_begin = std::chrono::high_resolution_clock::now();
    }

    cv::RotatedRect left_light = lightInfos[0], right_light = lightInfos[1];
    if(left_light.center.x > right_light.center.x) std::swap(left_light, right_light);
    auto vertices = new cv::Point2f[4];
    left_light.points(vertices);
    lights_position.push_back(cv::Point2f(tl_.x + (vertices[0].x + vertices[3].x) / 2, tl_.y + (vertices[0].y + vertices[3].y) / 2));
    lights_position.push_back(cv::Point2f(tl_.x + (vertices[1].x + vertices[2].x) / 2, tl_.y + (vertices[1].y + vertices[2].y) / 2));

    right_light.points(vertices);
    lights_position.push_back(cv::Point2f(tl_.x + (vertices[1].x + vertices[2].x) / 2, tl_.y + (vertices[1].y + vertices[2].y) / 2));
    lights_position.push_back(cv::Point2f(tl_.x + (vertices[0].x + vertices[3].x) / 2, tl_.y + (vertices[0].y + vertices[3].y) / 2));
    if(enable_debug_) drawLightInfo(lightInfos);

    if(enable_debug_) {
        ROS_WARN("end: %.4lf", std::chrono::duration<double, std::ratio<1, 1000000> >(std::chrono::high_resolution_clock::now() - detection_begin).count());
        detection_begin = std::chrono::high_resolution_clock::now();
    }

    return true;
}

cv::Mat ArmorProcess::separateColors(){
    std::vector<cv::Mat> channels;
    // 把一个3通道图像转换成3个单通道图像
    split(_roiImg,channels);//分离色彩通道

    if(enable_debug_) {
        imshow("split_B", channels[0]);
        imshow("split_G", channels[1]);
        imshow("split_R", channels[2]);
    }
    cv::Mat grayImg;

    //剔除我们不想要的颜色
    //对于图像中红色的物体来说，其rgb分量中r的值最大，g和b在理想情况下应该是0，同理蓝色物体的b分量应该最大,将不想要的颜色减去，剩下的就是我们想要的颜色
    if(use_red_enermy) {
        grayImg=channels.at(1);//R-B
    }else{
        //if(armor_color_== 1) grayImg=channels.at(2)-channels.at(0);//R-B
        //else grayImg=channels.at(0)-channels.at(2);//B-R
        grayImg=channels.at(1);
    }
    return grayImg;
}

void ArmorProcess::filterContours(std::vector<std::vector<cv::Point> >& lightContours, std::vector<cv::RotatedRect>& lightInfos){
    int num = lightContours.size();
    for(int i = 0; i < num; i++){
        std::vector<cv::Point> &contour = lightContours[i];
        if(lightContours[i].size() < 10) continue;
        //RotatedRect contour = lightContours[i]
        //得到面积
        float lightContourArea = contourArea(contour);
        //面积太小的不要
        if(lightContourArea < light_min_area) continue;
        //椭圆拟合区域得到外接矩形
        cv::RotatedRect lightRec = fitEllipse(contour);
        //矫正灯条的角度，将其约束为-45~45
        adjustRec(lightRec);
        //宽高比、凸度筛选灯条  注：凸度=轮廓面积/外接矩形面积
        //if(lightRec.size.width / lightRec.size.height > light_max_ratio || lightContourArea / lightRec.size.area() < light_contour_min_solidity) continue;
        //因为颜色通道相减后己方灯条直接过滤，不需要判断颜色了,可以直接将灯条保存
        lightInfos.push_back(lightRec);
    }
}

//绘制旋转矩形
void ArmorProcess::drawLightInfo(std::vector<cv::RotatedRect>& LightInfos){
    _debugImg = _roiImg.clone();
    std::vector<std::vector<cv::Point> > cons;
    int num = LightInfos.size();
    for(int i = 0; i < num; i++){
        cv::RotatedRect &rotate = LightInfos[i];
        auto vertices = new cv::Point2f[4];
        rotate.points(vertices);

        std::vector<cv::Point> con;
        for(int i = 0; i < 4; i++) con.push_back(vertices[i]);
        cons.push_back(con);
        drawContours(_debugImg, cons, i, cv::Scalar(0,255,255), 1,8);
        if(enable_debug_) imshow("rotateRectangle", _debugImg);
    }
}

//匹配灯条，筛选出装甲板
bool ArmorProcess::matchArmor(std::vector<cv::RotatedRect>& lightInfos){
    int num = lightInfos.size();
    std::vector<cv::RotatedRect> armors;
    //按灯条中心x从小到大排序
    if(num < 2) return false;
    if(num == 2) return true;

    sort(lightInfos.begin(), lightInfos.end(), [](const cv::RotatedRect& ld1, const cv::RotatedRect& ld2){
        //Lambda函数,作为sort的cmp函数
        return ld1.center.x < ld2.center.x;
    });
    for(size_t i = 0; i < lightInfos.size(); i++){
    //遍历所有灯条进行匹配
        for(size_t j = i + 1; (j < lightInfos.size()); j++){
            const cv::RotatedRect& leftLight  = lightInfos[i];
            const cv::RotatedRect& rightLight = lightInfos[j];

            //角差
            float angleDiff_ = std::abs(leftLight.angle - rightLight.angle);
            //长度差比率
            float LenDiff_ratio = std::abs(leftLight.size.height - rightLight.size.height) / std::max(leftLight.size.height, rightLight.size.height);
            //筛选
            //std::cout << i << " " << j << " " << angleDiff_ << " " << LenDiff_ratio << "\n";
            if(angleDiff_ > light_max_angle_diff_ || LenDiff_ratio > light_max_height_diff_ratio_){
                continue;
            }
            //左右灯条相距距离
            float dis = distance(leftLight.center, rightLight.center);
            //左右灯条长度的平均值
            float meanLen = (leftLight.size.height + rightLight.size.height) / 2;
            //左右灯条中心点y的差值
            float yDiff = std::abs(leftLight.center.y - rightLight.center.y);
            //y差比率
            float yDiff_ratio = yDiff / meanLen;
            //左右灯条中心点x的差值
            float xDiff = std::abs(leftLight.center.x - rightLight.center.x);
            //x差比率
            float xDiff_ratio = xDiff / meanLen;
            //相距距离与灯条长度比值
            float ratio = dis / meanLen;
            //筛选
            //std::cout << i << " " << j << " " << yDiff_ratio << " " <<  xDiff_ratio << " " << ratio << "\n";
            if(xDiff < light_min_dis_ || yDiff_ratio > light_max_y_diff_ratio_ || xDiff_ratio < light_min_x_diff_ratio_ || ratio > armor_max_aspect_ratio_ || ratio < armor_min_aspect_ratio_){
                continue;
            }

            cv::RotatedRect resultLeftLight = leftLight, resultRightLight = rightLight;
            lightInfos.resize(2);
            lightInfos[0] = resultLeftLight, lightInfos[1] = resultRightLight;
            return true;
        }
    }
    return false;
}

void ArmorProcess::adjustRec(cv::RotatedRect& rec) {
    float& width = rec.size.width;
    float& height = rec.size.height;
    float& angle = rec.angle;
    while(angle >= 90.0) angle -= 180.0;
    while(angle < -90.0) angle += 180.0;

    if(angle >= 45.0) {
        std::swap(width, height);
        angle -= 90.0;
    }
    else if(angle < -45.0) {
        std::swap(width, height);
        angle += 90.0;
    }
}