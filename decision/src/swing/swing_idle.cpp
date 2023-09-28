/*
 * @Author: your name
 * @Date: 2022-04-01 15:07:27
 * @LastEditTime: 2022-05-21 22:18:55
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: \roborts_ws\src\test_pkg\src\skills\swing_idle.cpp
 */
#include "./swing_idle.h"


using namespace ENUM_CLASS;

/**
 ** tick() 函数 可以理解为 Run()函数
 ** _halt_requested 是停止条件，
 ** 主动或者自动修改_halt_requested均可实现该ActionNode的终止
 */
BT::NodeStatus SwingIdle::tick()
{
	std::cout << "swing idle" << std::endl;
	_halt_requested.store(false);
	this->chassis_executor_->Cancel();

	SWING_STATUS swing_status = blackboard_ptr->ComputeSwingStatus();

	ros::Rate loop_rate(50);

	uint8_t cnts = 0;
	bool first = true;

	// //* 零速度
	// geometry_msgs::Twist twist_zero;
	// twist_zero.linear.x = 0;
	// twist_zero.linear.y = 0;
	// twist_zero.angular.z = 0;
    // 原地旋转消息
    geometry_msgs::Twist twist;
	roborts_msgs::TwistAccel twist_accl;
	twist_accl = this->twist_accl_zero;
	geometry_msgs::Twist current_twist;

	roborts_msgs::GimbalAngle gimbal_angle_msg;

    twist.linear.x = 0;
	twist.linear.y = 0;
	twist.angular.z = twist_angle;
	
    switch (swing_status)
    {
    case SWING_STATUS::SLOW:
        break;
    case SWING_STATUS::MIDDLE:
        twist.angular.z = twist_angle * 1.5;
        break;
    case SWING_STATUS::FAST:
        twist.angular.z = twist_angle * 2.0;
        break;
    }

	//* 即时速度
	current_twist = twist;

	gimbal_angle_msg.yaw_mode = true;
	gimbal_angle_msg.pitch_mode = true;
	gimbal_angle_msg.pitch_angle = 0.0;

	while (!_halt_requested && ros::ok()) {
		
		if (cnts == (this->period -20)) {
			current_twist = twist;
		}
		if (cnts < 20) {
			//* 慢起步
			current_twist.angular.z = 0.05 * cnts * twist.angular.z;
		}
		else if (cnts < (this->period -20) ){
			current_twist = twist;
		}
		else {
			//* 缓停步
			current_twist.angular.z = 0.05 * (this->period - 1 - cnts) * twist.angular.z;
		}
		///* 底盘旋转的负方向
		gimbal_angle_msg.yaw_angle = -current_twist.angular.z * this->ratio;

		//* 发布控制命令
		twist_accl.twist = current_twist;
		this->gimbal_executor_->Execute(gimbal_angle_msg);
		//* 
		//this->chassis_executor_->Execute(current_twist);
		this->chassis_executor_->Execute(twist_accl);

		cnts = cnts + 1;
		
		//* 重置cnts 且反向angular.z
		if (cnts == this->period) {
			twist_accl = this->twist_accl_zero;
			this->chassis_executor_->Execute(twist_accl);
			gimbal_angle_msg.yaw_angle = 0.0;
			this->gimbal_executor_->Execute(gimbal_angle_msg);

			twist.angular.z = first ? -2 * twist.angular.z : -twist.angular.z;
			//* 初次结束
			first = false;
			cnts = 0;
		}
        
        loop_rate.sleep();

        //* 挂起线程，tick()返回running
        CoroActionNode::setStatusRunningAndYield();
    }

	this->stop();
	std::cout << "swing idle end " << std::endl;

    return (_halt_requested ) ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

//* halt函数实现
void SwingIdle::halt()
{
	this->stop();
    CoroActionNode::halt();
}

void SwingIdle::init(ChassisExecutor *&chassis_executor, GimbalExecutor *&gimbal_executor){
	this->chassis_executor_ = chassis_executor;
	this->gimbal_executor_ = gimbal_executor;
}

void SwingIdle::stop() {
	this->_halt_requested.store(true);
	this->gimbal_executor_->Cancel();
	this->chassis_executor_->Cancel();
}