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

#include "mv_driver.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

namespace roborts_camera {
MVDriver::MVDriver(CameraInfo camera_info) : CameraBase(camera_info) {
  if (!MVDriver::sdk_inited_) {
    CameraSdkInit(1);
    MVDriver::sdk_inited_ = true;
    MVDriver::i_camera_counts_ = CameraEnumerateDeviceEx();
    ROS_INFO("MindVision camera count is %d\n", MVDriver::i_camera_counts_);
    // no device
    if (MVDriver::i_camera_counts_ == 0) {
      ROS_INFO("Couldn't find any MindVision camera!");
      return;
    }
  }
  if (!camera_initialized_) {
    if (CameraInitEx2((char *)camera_info.camera_name.c_str(), &h_camera_) !=
        CAMERA_STATUS_SUCCESS) {
      ROS_INFO("Failed to initialize MindVision camera");
      ROS_INFO(camera_info.camera_name.c_str());
      return;
    }
    CameraGetCapability(h_camera_, &t_capability_);
    g_pRgbBuffer_ =
        (unsigned char *)malloc(t_capability_.sResolutionRange.iHeightMax *
                                t_capability_.sResolutionRange.iWidthMax * 3);
    CameraPlay(h_camera_);
    CameraReadParameterFromFile(h_camera_, "/home/dji/roborts_ws/src/roborts_camera/config/bph.Config");
    /* 其他的相机参数设置，如
      CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
      CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
      CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
      更多的参数的设置方法，，清参考MindVision_Demo。本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
    */
    if (t_capability_.sIspCapacity.bMonoSensor) {
      CameraSetIspOutFormat(h_camera_, CAMERA_MEDIA_TYPE_MONO8);
    } else {
      CameraSetIspOutFormat(h_camera_, CAMERA_MEDIA_TYPE_BGR8);
    }
    camera_initialized_ = true;
  }
}

void MVDriver::StartReadCamera(cv::Mat &img) {
  if (CameraGetImageBuffer(h_camera_, &s_frame_info_, &pbyBuffer_, 1000) ==
      CAMERA_STATUS_SUCCESS) {
    CameraImageProcess(h_camera_, pbyBuffer_, g_pRgbBuffer_, &s_frame_info_);
    cv::Mat image;
    img = cv::Mat(s_frame_info_.iHeight, s_frame_info_.iWidth, CV_8UC3,
                  g_pRgbBuffer_, cv::Mat::AUTO_STEP);
    CameraReleaseImageBuffer(h_camera_, pbyBuffer_);
  }
}

void MVDriver::StopReadCamera() {
  // TODO: To be implemented
}

void MVDriver::SetCameraExposure(std::string id, int val) {
  
}

MVDriver::~MVDriver() {
  CameraUnInit(h_camera_);
  free(g_pRgbBuffer_);
}
}// namespace roborts_camera
