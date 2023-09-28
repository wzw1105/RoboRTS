/*
 * @Author: your name
 * @Date: 2022-04-05 20:43:56
 * @LastEditTime: 2022-05-21 21:26:21
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置
 * @FilePath: \roborts_ws\src\test_pkg\src\skills\swing_idle.h
 */
#ifndef _SWING_IDLE_H_
#define _SWING_IDLE_H_

#include "enum_class.h"
#include "share_head.h"
#include <nav_msgs/Odometry.h>
#include <mutex>

using namespace BT;
using namespace RTS_DECISION;



/**
 * ! 摇摆休息
 * * 原地摇摆
 */
class SwingIdle : public CoroActionNode {
  public:
    SwingIdle(const std::string& name):
        CoroActionNode(name, {})
    {
      //* 退出标志
      _halt_requested.store(false);
      this->swing_status = SWING_STATUS::SLOW;
      this->period = 50;
      this->twist_angle =  100.0 /180 * M_PI;
      geometry_msgs::Twist twist_zero;
      twist_zero.linear.x = 0;
      twist_zero.linear.y = 0;
      twist_zero.angular.z = 0;

      this->twist_accl_zero.twist = twist_zero;
      this->twist_accl_zero.accel.angular.z = 0;
      this->twist_accl_zero.accel.linear.x = 0;
      this->twist_accl_zero.accel.linear.y = 0;
      this->ratio = 0.0195;
    }

    BT::NodeStatus tick() override;
    virtual void halt() override;

    void init(ChassisExecutor *&chassis_executor, GimbalExecutor *&gimbal_executor);

    void stop();

  private:
    //* 退出标志
    std::atomic_bool _halt_requested;

    SWING_STATUS swing_status;
    double twist_angle;
    ChassisExecutor *chassis_executor_;
    GimbalExecutor *gimbal_executor_;
    roborts_msgs::TwistAccel twist_accl_zero;
    uint8_t period; //* 周期
    double ratio;
};

#endif