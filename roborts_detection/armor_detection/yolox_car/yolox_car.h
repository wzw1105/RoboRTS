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

#ifndef ROBORTS_DETECTION_ARMOR_DETECTION_YOLOX_CAR_H
#define ROBORTS_DETECTION_ARMOR_DETECTION_YOLOX_CAR_H

//system include
#include <vector>
#include <list>
#include <string>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "alg_factory/algorithm_factory.h"
#include "state/error_code.h"
#include "cv_toolbox.h"
#include "../armor_detection_base.h"
#include "roborts_msgs/ArmorMsg.h"
#include "roborts_msgs/ArmorMsgs.h"
#include "armor_process.h"

namespace roborts_detection {

using roborts_common::ErrorCode;
using roborts_common::ErrorInfo;

enum State {
  INITIALIZED = 0,
  RUNNING = 1,
  FAILED = 2,
  STOPED = 3
};

/**
 *  This class describes the armor information, including maximum bounding box, vertex, standard deviation.
 */
class ArmorInfo {
 public:
  ArmorInfo(cv::Rect armor_rect, std::vector<cv::Point2f> armor_vertex, float armor_stddev = 0.0, int armor_color = -1, int armor_id = -1, int car_pose = -1) {
    rect = armor_rect;
    vertex = armor_vertex;
    stddev = armor_stddev;
    color = armor_color;
    id = armor_id;
    car_pose = car_pose;
  }
 public:
  cv::Rect rect;
  std::vector<cv::Point2f> vertex;
  float stddev;
  int color; // 0 blue 1 red
  int id;
  int car_pose;
};

enum EnemyColor {
  BLUE = 0, RED  = 1
};

/**
 * @brief This class achieved functions that can help to detect armors of RoboMaster vehicle.
 */
class YOLOXCar : public ArmorDetectionBase {
 public:
  YOLOXCar(std::shared_ptr<CVToolbox> cv_toolbox);
  /**
   * @brief Loading parameters from .prototxt file.
   */
  void LoadParam() override;
  /**
   * @brief The entrance function of armor detection.
   * @param translation Translation information of the armor relative to the camera.
   * @param rotation Rotation information of the armor relative to the camera.
   */
  ErrorInfo DetectArmor(bool &detected, cv::Point3f &target_3d) override;
  /**
   * @brief Detecting lights on the armors.
   * @param src Input image
   * @param lights Output lights information
   */

  ArmorInfo SlectFinalArmor(std::vector<ArmorInfo> &armors);
  /**
   *
   * @param armor
   * @param distance
   * @param pitch
   * @param yaw
   * @param bullet_speed
   */
  void CalcControlInfo(const std::vector<cv::Point2f> &vertex, std::vector<cv::Point3f> &target_3ds);

  /**
   * @brief Using two lights(left light and right light) to calculate four points of armor.
   * @param armor_points Out put
   * @param left_light Rotated rect of left light
   * @param right_light Rotated rectangles of right light
   */
  void SolveArmorCoordinate();

  void SignalFilter(double &new_num, double &old_num,unsigned int &filter_count, double max_diff);

  void SetThreadState(bool thread_state) override;

  void dilate_rectangle(cv::Point2f &tl, cv::Point2f &br, int img_cols, int img_rows);

  /**
   * @brief Destructor
   */
  ~YOLOXCar() final;
 private:
  ErrorInfo error_info_;
  int BLUE = 0, RED = 1;
  cv::Mat src_img_;
  cv::Mat gray_img_;
  //!  Camera intrinsic matrix
  cv::Mat intrinsic_matrix_;
  //! Camera distortion Coefficient
  cv::Mat distortion_coeffs_;
  //! Read image index
  int read_index_;
  //! detection time
  std::chrono::high_resolution_clock::time_point detection_begin;
  double detection_time_;
  double one_cycle_total_time_;

  //! armor info
  std::vector<cv::Point3f> armor_points_;

  bool thread_running_;
  bool enable_debug_;

  ArmorProcess armor_image;
  std::vector<ArmorInfo> armors;

  // param
  //use conventional methods for next image detection
  float dilate_width_ = 60;
  float dilate_height_ = 10;
  const int armor_width_ = 160;
  const int light_height_ = 55;

  //current step
  int cur_step = 0;
  //the length of the cycle;
  int cycle_length_ = 5;

  float color_thread_;
  float blue_thread_;
  float red_thread_;
  bool using_hsv_;

  int fps;

  

  //ros
  ros::NodeHandle nh;
  ros::Publisher armors_pub_;
};

roborts_common::REGISTER_ALGORITHM(ArmorDetectionBase, "yolox_car", YOLOXCar, std::shared_ptr<CVToolbox>);

} //namespace roborts_detection

#endif // AOTO_PILOT_DETECTION_ARMOR_DETECTION_YOLOX_CAR_H
