/*
 * @Author: kidding 2867634589@qq.com
 * @Date: 2022-04-27 20:04:18
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2022-05-21 16:18:31
 * @FilePath: /roborts_ws/src/RoboRTS/decision/src/executor/gimbal_executor.h
 * @Description: 这是默认设置,请设置`customMade`
 */
#ifndef ROBORTS_DECISION_GIMBAL_EXECUTOR_H
#define ROBORTS_DECISION_GIMBAL_EXECUTOR_H
#include "ros/ros.h"

#include <roborts_msgs/GimbalAngle.h>
#include <roborts_msgs/ShootCmd.h>
#include <roborts_msgs/FricWhl.h>

#include "enum_class.h"
using namespace ENUM_CLASS;

namespace RTS_DECISION{
/***
 * @brief Gimbal Executor to execute different abstracted task for gimbal module
 */
class GimbalExecutor{
 public:
  /**
   * @brief Constructor of GimbalExecutor
   */
  GimbalExecutor();
  ~GimbalExecutor() = default;
  /***
   * @brief Execute the gimbal angle task with publisher
   * @param gimbal_angle Given gimbal angle
   */
  void Execute(const roborts_msgs::GimbalAngle &gimbal_angle);
  
  /***
   * @brief Execute the shoot task with publisher
   * @param shoot_cmd Given shoot_cmd
   * ! 服务调用，会修改shoot_cmd中的值，其中的received值 会被修改为ture or false
   */
  void Execute(roborts_msgs::ShootCmd &shoot_cmd);

  /**
  * @brief 唤醒摩擦轮
  * * 唤醒摩擦轮
  * @param fricwhl 
  */
  void Execute(roborts_msgs::FricWhl &fricwhl);

  /**
   * @brief Update the current gimbal executor state
   * @return Current gimbal executor state(same with behavior state)
   */
  BehaviorState Update();
  /**
   * @brief Cancel the current task and deal with the mode transition
   */
  void Cancel();

 private:
  //! execution mode of the executor
  GimbalExcutionMode excution_mode_;
  //! execution state of the executor (same with behavior state)
  BehaviorState execution_state_;
  //! no shoot
  roborts_msgs::ShootCmd no_shoot;

  //! zero gimbal angle in form of ROS roborts_msgs::GimbalAngle
  roborts_msgs::GimbalAngle zero_gimbal_angle_;

  //! gimbal angle control publisher in ROS
  ros::Publisher cmd_gimbal_angle_pub_;

  //! shooter control client in ROS
  ros::ServiceClient shoot_service_client_;

  //! Call FricWhl 
  ros::ServiceClient FricWhl_service_client_;
};
}


#endif //ROBORTS_DECISION_GIMBAL_EXECUTOR_H