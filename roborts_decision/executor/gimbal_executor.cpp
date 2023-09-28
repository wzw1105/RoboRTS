#include "gimbal_executor.h"

namespace roborts_decision{
GimbalExecutor::GimbalExecutor():excution_mode_(ExcutionMode::IDLE_MODE),
                                 execution_state_(BehaviorState::IDLE){
  ros::NodeHandle nh;
  cmd_gimbal_angle_pub_ = nh.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 1);

  zero_gimbal_angle_.yaw_mode = false;
  zero_gimbal_angle_.pitch_mode = false;
  zero_gimbal_angle_.yaw_angle = 0;
  zero_gimbal_angle_.pitch_angle = 0;

}

void GimbalExecutor::Execute(const roborts_msgs::GimbalAngle &gimbal_angle){
  excution_mode_ = ExcutionMode::ANGLE_MODE;
  cmd_gimbal_angle_pub_.publish(gimbal_angle);
}

BehaviorState GimbalExecutor::Update(){
  switch (excution_mode_){
    case ExcutionMode::IDLE_MODE:
      execution_state_ = BehaviorState::IDLE;
      break;

    case ExcutionMode::ANGLE_MODE:
      execution_state_ = BehaviorState::RUNNING;
      break;

    case ExcutionMode::RATE_MODE:
      execution_state_ = BehaviorState::RUNNING;
      break;

    default:
      ROS_ERROR("Wrong Execution Mode");
  }
  return execution_state_;
}

void GimbalExecutor::Cancel(){
  switch (excution_mode_){
    case ExcutionMode::IDLE_MODE:
      ROS_WARN("Nothing to be canceled.");
      break;

    case ExcutionMode::ANGLE_MODE:
      cmd_gimbal_angle_pub_.publish(zero_gimbal_angle_);
      excution_mode_ = ExcutionMode::IDLE_MODE;
      break;

    default:
      ROS_ERROR("Wrong Execution Mode");
  }

}
}