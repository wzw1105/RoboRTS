#include <ros/ros.h>
#include "yolox_s_car.h"
int main(int argc, char **argv) {
    ROS_INFO("test cmake!");
    init_model("/home/dji/roborts_ws/src/roborts_detection/armor_detection/yolox_s_car_v1_0_test/params/yolox_s_car_v1_1.param", \
               "/home/dji/roborts_ws/src/roborts_detection/armor_detection/yolox_s_car_v1_0_test/params/yolox_s_car_v1_1.bin");
    ROS_INFO("yolox inited!");
    cv::Mat m = cv::imread("/home/dji/roborts_cv/frame.jpg", 1);
    ROS_INFO("image loaded!");
    std::vector<Object> objects;
    detect_yolox(m, objects);
    ROS_INFO("object detected!");
    draw_objects(m, objects);
    return 0;
}
