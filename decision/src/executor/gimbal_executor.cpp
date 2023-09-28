/*
 * @Author: kidding 2867634589@qq.com
 * @Date: 2022-04-27 20:04:18
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2022-05-21 16:35:41
 * @FilePath: /roborts_ws/src/RoboRTS/decision/src/executor/gimbal_executor.cpp
 * @Description: 这是默认设置,请设置`customMade`
 */
#include "./gimbal_executor.h"

namespace RTS_DECISION{
  
GimbalExecutor::GimbalExecutor():excution_mode_(GimbalExcutionMode::IDLE_MODE),
                                 execution_state_(BehaviorState::IDLE){
  ros::NodeHandle nh;
  cmd_gimbal_angle_pub_ = nh.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 1);
  shoot_service_client_ = nh.serviceClient<roborts_msgs::ShootCmd>("cmd_shoot");
  FricWhl_service_client_ = nh.serviceClient<roborts_msgs::FricWhl>("cmd_fric_wheel");
  zero_gimbal_angle_.yaw_mode = true;
  zero_gimbal_angle_.yaw_angle = 0;
  zero_gimbal_angle_.pitch_mode = true;
  zero_gimbal_angle_.pitch_angle = 0;
  //* 停止射击
  no_shoot.request.mode = 0;
  no_shoot.request.number = 0;
}

/***
  * @brief Execute the gimbal angle task with publisher
  * @param gimbal_angle Given gimbal angle
  */
void GimbalExecutor::Execute(const roborts_msgs::GimbalAngle &gimbal_angle){
  excution_mode_ = GimbalExcutionMode::ANGLE_MODE;
  cmd_gimbal_angle_pub_.publish(gimbal_angle);
}

/***
  * @brief Execute the shoot task with publisher
  * @param shoot_cmd Given shoot_cmd
  * ! 服务调用，会修改shoot_cmd中的值，其中的received值 会被修改为ture or false
  * * 因此不能用const 修饰参数
  */
void GimbalExecutor::Execute(roborts_msgs::ShootCmd &shoot_cmd){
  excution_mode_ = GimbalExcutionMode::SHOOT_MODE;
  shoot_service_client_.call(shoot_cmd);
}

/**
  * @brief 唤醒摩擦轮
  * * 唤醒摩擦轮
  * @param fricwhl 
*/
void GimbalExecutor::Execute(roborts_msgs::FricWhl &fricwhl){
  //* 唤醒摩擦轮
  this->FricWhl_service_client_.call(fricwhl);
}

/**
  * @brief Update the current gimbal executor state
  * @return Current gimbal executor state(same with behavior state)
  */
BehaviorState GimbalExecutor::Update(){
  switch (excution_mode_){
    case GimbalExcutionMode::IDLE_MODE:
      execution_state_ = BehaviorState::IDLE;
      break;
    case GimbalExcutionMode::ANGLE_MODE:
      execution_state_ = BehaviorState::RUNNING;
      break;
    case GimbalExcutionMode::SHOOT_MODE:
      execution_state_ = BehaviorState::RUNNING;
      break;

    default:
      ROS_ERROR("Wrong Execution Mode");
  }
  return execution_state_;
}

void GimbalExecutor::Cancel(){
  switch (excution_mode_){
    case GimbalExcutionMode::IDLE_MODE:
      ROS_WARN("Nothing to be canceled.");
      break;

    case GimbalExcutionMode::ANGLE_MODE:
      zero_gimbal_angle_.yaw_mode = false;
      cmd_gimbal_angle_pub_.publish(zero_gimbal_angle_);
      shoot_service_client_.call(no_shoot);
      excution_mode_ = GimbalExcutionMode::IDLE_MODE;
      break;

    case GimbalExcutionMode::SHOOT_MODE:
      shoot_service_client_.call(no_shoot);
      zero_gimbal_angle_.yaw_mode = false;
      cmd_gimbal_angle_pub_.publish(zero_gimbal_angle_);
      excution_mode_ = GimbalExcutionMode::IDLE_MODE;
      break;

    default:
      ROS_ERROR("Wrong Execution Mode");
  }

}


}