/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <ros/ros.h>
#include "yolox_car.h"
#include "yolox_s_car.h"
#include "io/io.h"
#include "timer/timer.h"
#include <cstdio>

namespace roborts_detection {

YOLOXCar::YOLOXCar(std::shared_ptr<CVToolbox> cv_toolbox)
    : ArmorDetectionBase(cv_toolbox) {
    
    nh.param<bool>("enable_debug_", enable_debug_, false);
    nh.param<int>("cycle_length_", cycle_length_, 5);
    nh.param<float>("dilate_width_", dilate_width_, 60);
    nh.param<float>("dilate_height_", dilate_height_, 10);

    read_index_ = -1;
    detection_time_ = 0;
    thread_running_ = false;

    color_thread_ = 100;
    blue_thread_ = 90;
    red_thread_ = 50;
    using_hsv_ = false;

    SolveArmorCoordinate();  // tl tr bl br 
    // init model
    init_model("/home/dji/roborts_ws/src/roborts_detection/armor_detection/yolox_car/params/yolox_s_car_v2_0.param", \
               "/home/dji/roborts_ws/src/roborts_detection/armor_detection/yolox_car/params/yolox_s_car_v2_0.bin");

    LoadParam();
    error_info_ = ErrorInfo(roborts_common::OK);
    armors_pub_ = nh.advertise<roborts_msgs::ArmorMsgs>("armors_info", 10);
}

void YOLOXCar::LoadParam() {
    // read parameters
    int get_intrinsic_state = -1;
    int get_distortion_state = -1;

    while ((get_intrinsic_state < 0) || (get_distortion_state < 0)) {
        ROS_WARN("Wait for camera driver launch %d", get_intrinsic_state);
        usleep(50000);
        ros::spinOnce();
        get_intrinsic_state = cv_toolbox_->GetCameraMatrix(intrinsic_matrix_);
        get_distortion_state =
            cv_toolbox_->GetCameraDistortion(distortion_coeffs_);
    }
}

void YOLOXCar::dilate_rectangle(cv::Point2f &tl, cv::Point2f &br, int img_cols, int img_rows) {
    tl.x = std::max(0.0f, tl.x - dilate_width_), tl.y = std::max(0.0f, tl.y - dilate_height_);
    br.x = std::min((float)(img_cols - 1), br.x + dilate_width_), br.y = std::min((float)(img_rows - 1), br.y + dilate_height_);
}

ErrorInfo YOLOXCar::DetectArmor(bool &armor_detected, cv::Point3f &target_3d) {
    std::vector<Object> cars;
    std::vector<Object> objects;
    auto img_begin = std::chrono::high_resolution_clock::now();
    bool sleep_by_diff_flag = true;
    char path[100];
    while (true) {
        // Ensure exit this thread while call Ctrl-C
        if (!thread_running_) {
            ErrorInfo error_info(ErrorCode::STOP_DETECTION);
            return error_info;
        }
        read_index_ = cv_toolbox_->NextImage(src_img_);
        if (read_index_ < 0) {
            // Reducing lock and unlock when accessing function 'NextImage'
            if (detection_time_ == 0) {
                usleep(20000);
                continue;
            } else {
                double capture_time = 0;
                cv_toolbox_->GetCaptureTime(capture_time);
                if (capture_time == 0) {
                    // Make sure the driver is launched and the image callback
                    // is called
                    usleep(20000);
                    continue;
                } else if (capture_time > detection_time_ &&
                           sleep_by_diff_flag) {
                    // ROS_WARN("time sleep %lf", (capture_time -
                    //          detection_time_));
                    usleep((unsigned int)(capture_time - detection_time_));
                    sleep_by_diff_flag = false;
                    continue;
                } else {
                    // For real time request when image call back called, the
                    // function 'NextImage' should be called.
                    usleep(500);
                    continue;
                }
            }
        } else {
            break;
        }
    }
    detection_begin = std::chrono::high_resolution_clock::now();
    auto tmp = std::chrono::high_resolution_clock::now();
    if(enable_debug_) ROS_INFO("current step: %d", cur_step);
    // cv::imshow("src_img_", src_img_);
    if(cur_step == 0) {
        //clear
        
        armors.clear();
        detect_yolox(src_img_, objects);
        if(enable_debug_) ROS_INFO("yolox detection time: %.4lf", std::chrono::duration<double, std::ratio<1, 1000000> >(std::chrono::high_resolution_clock::now() - detection_begin).count());
        one_cycle_total_time_ = 0;
        // "armor_blue_1", "armor_blue_2", "armor_red_1", "armor_red_2"
        // "car_back", "car_front", "car_incline", "car_left", "car_right"
        for(auto object : objects) if(object.label >= 4) cars.push_back(object);
        for(auto object : objects) {
            if (object.label < 4) {
                //check whether an armor in a car
                bool in_car = false;
                int car_pose = -1;
                for(auto car : cars) {
                    cv::Point2f car_tl = car.rect.tl(), car_br = car.rect.br(), armor_tl = object.rect.tl(), armor_br = object.rect.br();
                    if(car_tl.x < armor_tl.x && car_tl.y < armor_tl.y && car_br.x > armor_br.x && car_br.y > armor_br.y) {
                        in_car = true;
                        // front 1, left 2, back 3, right 4, incline 0
                        if (car.label == 5) car_pose = 1;
                        else if (car.label == 7) car.label = 2;
                        else if (car.label == 4) car.label = 3;
                        else if (car.label == 8) car.label = 4;
                        else if (car.label == 6) car.label = 0;
                        break;
                    }
                }
                if(!in_car) continue;
                // tl tr br bl
                cv::Point2f tl = object.rect.tl(), br = object.rect.br();
                dilate_rectangle(tl, br, (float)src_img_.cols, (float)src_img_.rows);
                //tl.x = std::max(0.0f, tl.x - diltate_length_), br.x = std::min((float)src_img_.cols, br.x + diltate_length_);
                cv::Point2f bl(tl.x, br.y), tr(br.x, tl.y);

                int color = object.label < 2 ? 0 : 1;
                int id = object.label & 1 ? 2 : 1;

                cv::Rect final_rect(tl.x, tl.y, tr.x - tl.x + 1, bl.y - tl.y + 1);
                armors.push_back(ArmorInfo(final_rect, std::vector<cv::Point2f>({bl, tl, tr, br}), 0.0, color, id, car_pose));
            }
        }
    }else{
        //dilate each armor
        for(auto &armor : armors) {
            cv::Rect pre_rect = armor.rect;
            cv::Point2f tl = pre_rect.tl(), br = pre_rect.br();
            dilate_rectangle(tl, br, (float)src_img_.cols, (float)src_img_.rows);
            cv::Point2f bl(tl.x, br.y), tr(br.x, tl.y);
            cv::Rect final_rect(tl.x, tl.y, tr.x - tl.x + 1, bl.y - tl.y + 1);
            armor = ArmorInfo(final_rect, std::vector<cv::Point2f>({bl, tl, tr, br}), 0.0, armor.color, armor.id, armor.car_pose);
        }
    }
    if(enable_debug_) ROS_INFO("step1 time: %.4lf", std::chrono::duration<double, std::ratio<1, 1000000> >(std::chrono::high_resolution_clock::now() - detection_begin).count());
    
    cv::Mat im2show = draw_objects(src_img_, objects);
    //cv::Mat im2show = src_img_.clone();
    
    const cv::Scalar colors[3] = {{255, 0, 0}, {0, 0, 255}, {0, 255, 0}}; //BGR
    std::vector<cv::Point3f> target_3ds;
    std::vector<int16_t> armors_area;

    for (std::vector<ArmorInfo>::iterator it = armors.begin(); it != armors.end();) {
        ArmorInfo cur_armor = *it;
        ROS_INFO("CUR_ARMOR COLOR: %d", cur_armor.color); 
        tmp = std::chrono::high_resolution_clock::now();
        armor_image.Init(cur_armor.color, src_img_, cur_armor.vertex[1], cur_armor.rect.width, cur_armor.rect.height);
        if(enable_debug_) ROS_INFO("Init time: %.4lf", std::chrono::duration<double, std::ratio<1, 1000000> >(std::chrono::high_resolution_clock::now() - tmp).count());
        tmp = std::chrono::high_resolution_clock::now();
        std::vector<cv::Point2f> light_position_;
        if(!armor_image.FindMatchLights(light_position_) || light_position_.size() < 4) {
            armors.erase(it);
            continue;
        }
        if(enable_debug_) ROS_INFO("Image process time: %.4lf", std::chrono::duration<double, std::ratio<1, 1000000> >(std::chrono::high_resolution_clock::now() - tmp).count());
        if(enable_debug_) {
            cv::line(im2show, cur_armor.vertex[0], cur_armor.vertex[1], colors[cur_armor.color], 2);
            cv::line(im2show, cur_armor.vertex[1], cur_armor.vertex[2], colors[cur_armor.color], 2);
            cv::line(im2show, cur_armor.vertex[2], cur_armor.vertex[3], colors[cur_armor.color], 2);
            cv::line(im2show, cur_armor.vertex[3], cur_armor.vertex[0], colors[cur_armor.color], 2);
        }

        CalcControlInfo(light_position_, target_3ds);
        if(enable_debug_) {
            cv::line(im2show, light_position_[0], light_position_[1], colors[cur_armor.color], 2);
            cv::line(im2show, light_position_[1], light_position_[2], colors[cur_armor.color], 2);
            cv::line(im2show, light_position_[2], light_position_[3], colors[cur_armor.color], 2);
            cv::line(im2show, light_position_[3], light_position_[0], colors[cur_armor.color], 2);
            cv::putText(im2show, (std::string)(cur_armor.color == 0 ? "BLUE" : "RED") + std::to_string(cur_armor.id), light_position_[1], cv::FONT_HERSHEY_SIMPLEX, 1, colors[cur_armor.color], 2, 8);
        }
        armors_area.emplace_back((int16_t)(fabs(light_position_[0].x - light_position_[3].x) * fabs(light_position_[0].y - light_position_[1].y) ));
        //update rect
        cv::Point2f bl = light_position_[0], tl = light_position_[1], tr = light_position_[2], br = light_position_[3];
        float new_tlx = std::min(bl.x, tl.x);
        float new_tly = std::min(std::max(0.0f, tl.y - (bl.y - tl.y)/2),  std::max(0.0f, tr.y - (br.y - tr.y)/2));
        float new_brx = std::max(br.x, tr.x);
        float new_bry = std::max(std::min((float)(src_img_.rows - 1), bl.y + (bl.y - tl.y)/2), std::min((float)(src_img_.rows - 1), br.y + (br.y - tr.y)/2));
        bl.x = new_tlx, bl.y = new_bry;
        tl.x = new_tlx, tl.y = new_tly;
        tr.x = new_brx, tr.y = new_tly;
        br.x = new_brx, tr.y = new_bry;
        cv::Rect final_rect(tl.x, tl.y, tr.x - tl.x + 1, bl.y - tl.y + 1);
        ROS_INFO("INIT_COLOR: %d", cur_armor.color);
        *it = ArmorInfo(final_rect, std::vector<cv::Point2f>({bl, tl, tr, br}), 0.0, cur_armor.color, cur_armor.id);

        it++;
    }

    if(enable_debug_) ROS_INFO("step2 time: %.4lf", std::chrono::duration<double, std::ratio<1, 1000000> >(std::chrono::high_resolution_clock::now() - detection_begin).count());
    //if(enable_debug_) std::cout << "armors, armors_area, target_3ds size: " << armors.size() << " " << armors_area.size() << " " << target_3ds.size() << "\n";

    // encapsulate message
    /* ArmorMsg
        bool robot_detected
        bool armor_detected
        int8 robot_pose
        int8 color
        int8 id
        geometry_msgs/Point pose */
    roborts_msgs::ArmorMsgs armor_msgs;
    std::vector<roborts_msgs::ArmorMsg> detected_info;
    if (armors.size()) {
        armor_msgs.detected = true;
        roborts_msgs::ArmorMsg armor_info;
        for (int i = 0; i < armors.size(); i++) {
            // checkout whether armor in robot
            /*armor_info.robot_detected = false;
            for (size_t j = 0; j < cars.size(); ++j) {
                // checkout whether detect robot
                cv::Rect inter_rect = (armors[i].rect) & ((cv::Rect)cars[j].rect);
                if (inter_rect.area() * 2 > armors[i].rect.area()) {
                    armor_info.robot_detected = true;
                    break;
                }
            }*/
            armor_info.armor_detected = true;
            armor_info.color = (armors[i].color + 1) % 2; // 0 red 1 blue
            armor_info.robot_pose = armors[i].car_pose;
            armor_info.id = armors[i].id;
            armor_info.area = armors_area[i];
            armor_info.pose.x = target_3ds[i].z / 1000;
            armor_info.pose.y = - target_3ds[i].x / 1000;
            armor_info.pose.z = - target_3ds[i].y / 1000;
            ROS_INFO("(x, y, z): (%lf, %lf, %lf)", armor_info.pose.x, armor_info.pose.y, armor_info.pose.z);
            detected_info.push_back(armor_info);
        }
        armor_msgs.detected = true;
    } else if (cars.size()) {
        armor_msgs.detected = true;
    }
    armor_msgs.detected_info = detected_info;
    armors_pub_.publish(armor_msgs);

    detection_time_ = std::chrono::duration<double, std::ratio<1, 1000000> >(std::chrono::high_resolution_clock::now() - detection_begin).count();
    ROS_INFO("this step time: %.4lf", detection_time_);
    one_cycle_total_time_ += detection_time_;
    ROS_INFO("current one_cycle_total_time_: %.4lf", one_cycle_total_time_);
    
    if (enable_debug_) {
        //sprintf(path, "/media/dji/KINGSTON/videos/img_%06d.jpg", cnt++);
        //cv::imshow("relust_img_", result_img_);
        if(cur_step == cycle_length_ - 1) {
            //detection_time_ = std::chrono::duration<double, std::ratio<1, 1000000> >(std::chrono::high_resolution_clock::now() - detection_begin).count();
            fps =  (int)(cycle_length_ * 1000000.0 / one_cycle_total_time_);
            std::cout << "FPS: " << fps << "\n";
        }
        cv::putText(im2show, (std::string)("FPS: ") + std::to_string(fps), cv::Point2f(0, 100), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);
        cv::imshow("im2show", im2show);
        //cv::imwrite(path, result_img_);
        cv::waitKey(5);
    }



    cv_toolbox_->ReadComplete(read_index_);
    if(enable_debug_) ROS_INFO("read complete\n");
    
    cur_step = (cur_step + 1) % cycle_length_;
    return error_info_;
}


ArmorInfo YOLOXCar::SlectFinalArmor(std::vector<ArmorInfo> &armors) {
    std::sort(armors.begin(), armors.end(), [](const ArmorInfo &p1, const ArmorInfo &p2) {
        return p1.rect.area() > p2.rect.area();
    });
    return armors[0];
}

void YOLOXCar::CalcControlInfo(const std::vector<cv::Point2f> &vertex, std::vector<cv::Point3f> &target_3ds) {
    cv::Mat rvec;
    cv::Mat tvec;
    cv::solvePnP(armor_points_, vertex, intrinsic_matrix_, distortion_coeffs_, rvec, tvec);

    cv::Point3f target_3d(tvec);
    cv::Point3f target_angle(rvec);

    target_3ds.emplace_back(target_3d);
}


void YOLOXCar::SolveArmorCoordinate() {
    //use four points of light for PnP
    armor_points_.emplace_back(cv::Point3f(-armor_width_ / 2, - light_height_ / 2, 0.0));
    armor_points_.emplace_back(cv::Point3f(-armor_width_ / 2, light_height_ / 2 , 0.0));
    armor_points_.emplace_back(cv::Point3f(armor_width_/ 2, light_height_ / 2 , 0.0));
    armor_points_.emplace_back(cv::Point3f(armor_width_ / 2, - light_height_ / 2, 0.0));
}

void YOLOXCar::SignalFilter(double &new_num, double &old_num, unsigned int &filter_count, double max_diff) {
    if (fabs(new_num - old_num) > max_diff && filter_count < 2) {
        filter_count++;
        new_num += max_diff;
    } else {
        filter_count = 0;
        old_num = new_num;
    }
}

void YOLOXCar::SetThreadState(bool thread_state) {
    thread_running_ = thread_state;
}

YOLOXCar::~YOLOXCar() {}
}  // namespace roborts_detection
