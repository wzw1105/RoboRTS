/*
 * @Author: your name
 * @Date: 2022-03-15 11:20:20
 * @LastEditTime: 2022-05-16 21:45:56
 * @LastEditors: Please set LastEditors
 * @Description: 
 * @FilePath: \roborts_ws\src\RoboRTS\test_pkg\src\skills\shoot_action.h
 */
#ifndef _SHOOT_ACTION_H_
#define _SHOOT_ACTION_H_

#include "enum_class.h"
#include "share_head.h"

using namespace BT;
using namespace RTS_DECISION;

class ShootAction : public AsyncActionNode
{
public:
	ShootAction(const std::string& name, const BT::NodeConfiguration& config):
        AsyncActionNode(name, config)
    {
	  this->_halt_requested.store(false);
	  this->seq = 0;

	  this->swing_status = SWING_STATUS::SLOW;
      this->period = 50;
      this->twist_angle =  100.0 /180 * M_PI;
      this->twist_zero.linear.x = 0;
      this->twist_zero.linear.y = 0;
      this->twist_zero.angular.z = 0;
      this->ratio = 0.0195;
	};
	~ShootAction();
	static BT::PortsList providedPorts();
	NodeStatus tick() override;
	virtual void halt() override;

	void init(ChassisExecutor*& chassis_executor, GimbalExecutor*& gimbal_executor);
		
private:
	std::atomic_bool _halt_requested;

	GimbalExecutor* gimbal_executor_;
	ChassisExecutor* chassis_executor_;
	uint8_t seq;

	SWING_STATUS swing_status;
    double twist_angle;
    double last_chassis_yaw;
    double odom_yaw_angle;
    geometry_msgs::Twist twist_zero;
    uint8_t period; //* 周期
	double ratio;

	void stop();

};



#endif