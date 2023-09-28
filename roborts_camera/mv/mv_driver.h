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

#ifndef ROBORTS_CAMERA_MV_DRIVER_H
#define ROBORTS_CAMERA_MV_DRIVER_H

#include <thread>
#include "../camera_base.h"
#include "../camera_param.h"
#include "actionlib/server/simple_action_server.h"
#include "alg_factory/algorithm_factory.h"
#include "io/io.h"
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "CameraApi.h"

namespace roborts_camera {
/**
 * @brief MV Camera class, product of the camera factory inherited from
 * CameraBase
 */
class MVDriver : public CameraBase {
 public:
  /**
   * @brief Constructor of MVDriver
   * @param camera_info  Information and parameters of camera
   */
  explicit MVDriver(CameraInfo camera_info);
  /**
   * @brief Start to read MV camera
   * @param img Image data in form of cv::Mat to be read
   */
  void StartReadCamera(cv::Mat &img) override;
  /**
   * @brief Stop to read MV camera
   */
  void StopReadCamera();
  ~MVDriver() override;

 private:
  /**
   * @brief Set camera exposure
   * @param id Camera path
   * @param val Camera exposure value
   */
  void SetCameraExposure(std::string id, int val);
  
  // mv camera
  static bool sdk_inited_;
  static int i_camera_counts_;

  bool read_camera_initialized_;
  int h_camera_;
  tSdkCameraDevInfo t_camera_info_;
  tSdkCameraCapbility t_capability_;      //设备描述信息
  tSdkFrameHead s_frame_info_;
  BYTE* pbyBuffer_;
  unsigned char* g_pRgbBuffer_;   // Data buffer
};

bool MVDriver::sdk_inited_ = false;
int MVDriver::i_camera_counts_ = 1;

roborts_common::REGISTER_ALGORITHM(CameraBase, "mv", MVDriver, CameraInfo);

}  // namespace roborts_camera
#endif  // ROBORTS_CAMERA_MV_DRIVER_H
