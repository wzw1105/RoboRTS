/*
 * @Author: your name
 * @Date: 2022-03-15 11:20:27
 * @LastEditTime: 2022-05-19 09:06:08
 * @LastEditors: Please set LastEditors
 * @Description: 
 * @FilePath: \roborts_ws\src\RoboRTS\test_pkg\src\skills\shoot_action.cpp
 */

#include "./shoot_action.h"

using namespace ENUM_CLASS;

ShootAction::~ShootAction(){}

NodeStatus ShootAction::tick(){
    std::cout<<"shoot start!" <<std::endl;
    this->_halt_requested.store(false);
    
    ros::Rate loop_rate(50);
    this->seq = blackboard_ptr->shoot_seq;
	uint8_t cnts = 0;
	bool first = true;

    this->chassis_executor_->Cancel();

    roborts_msgs::ShootCmd shoot_cmd;
    roborts_msgs::GimbalAngle gimbal_angle_msg;
    roborts_msgs::GimbalAngle gimbal_angle_adjust_msg_;
    //* 射击命令
    shoot_cmd.request.mode = shoot_cmd.request.ONCE;
	gimbal_angle_msg.yaw_mode = true;
	gimbal_angle_msg.pitch_mode=false;

    //* 射击时云台调整消息
    gimbal_angle_adjust_msg_.yaw_mode = true;
    gimbal_angle_adjust_msg_.pitch_mode = false;

    //* 原地旋转消息
    geometry_msgs::Twist twist;
	geometry_msgs::Twist current_twist;
    twist.linear.x = 0;
	twist.linear.y = 0;
	twist.angular.z = twist_angle;
	//* 即时速度
	current_twist = twist;
    //* 循环: 射击、扭腰
    while (!_halt_requested && ros::ok()) {
        // if (seq == blackboard_ptr->shoot_seq) {
        //     shoot_cmd.request.mode = shoot_cmd.request.STOP;
        // }
        // this->seq = blackboard_ptr->shoot_seq;

        if (blackboard_ptr->if_enemy_armor_detected && blackboard_ptr->enemy_distance < 2.5)
        {
            //* 从blackboard读取射击数量
            auto shoot_number_msg = getInput<uint8_t>("number");
            if (!shoot_number_msg)
            {
                throw BT::RuntimeError("missing required input [message]: ", shoot_number_msg.error());
                shoot_cmd.request.mode = shoot_cmd.request.STOP;
                shoot_cmd.request.number = 0;
            }
            else
            {   
                //* 从计算子弹数量的节点 获取射击子弹的数量
                shoot_cmd.request.mode = shoot_cmd.request.ONCE;
                shoot_cmd.request.number = shoot_number_msg.value();
                shoot_cmd.response.received = false;
            }
            
            // //* 从blackboard获取 yaw 和 pitch 的offset;
		    // {
            //     std::lock_guard<std::mutex> lock_guard(blackboard_ptr->mutex_gimbal_angle_);
            //     gimbal_angle_adjust_msg_.yaw_angle = blackboard_ptr->gimbal_yaw_offset;
            //     blackboard_ptr->gimbal_yaw_offset = 0.0;
		    // }
            // gimbal_angle_adjust_msg_.pitch_angle = blackboard_ptr->gimbal_pitch_offset;

            //* 调整云台、射击敌人
            gimbal_executor_->Execute(gimbal_angle_adjust_msg_);
            gimbal_executor_->Execute(shoot_cmd);

        } 
        else {
            //* 不适合射击（没检测到 或者 距离大于2.5米）
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

		    // //* 从blackboard获取 yaw 和 pitch 的offset;
		    // {
            //     std::lock_guard<std::mutex> lock_guard(blackboard_ptr->mutex_gimbal_angle_);
            //     gimbal_angle_msg.yaw_angle = gimbal_angle_msg.yaw_angle + blackboard_ptr->gimbal_yaw_offset;
            //     blackboard_ptr->gimbal_yaw_offset = 0.0;
		    // }
            // gimbal_angle_msg.pitch_angle = blackboard_ptr->gimbal_pitch_offset;

            gimbal_executor_->Execute(gimbal_angle_msg);
            chassis_executor_->Execute(current_twist);

            cnts = cnts + 1;

            //* 重置cnts 且反向angular.z
            if (cnts == this->period)
            {

                this->chassis_executor_->Execute(twist_zero);
                gimbal_angle_msg.yaw_angle = 0.0;
                this->gimbal_executor_->Execute(gimbal_angle_msg);

                twist.angular.z = first ? -2 * twist.angular.z : -twist.angular.z;
                //* 初次结束
                first = false;
                cnts = 0;
		    }
        }
        
        loop_rate.sleep();
    }

    this->stop();
    std::cout<<"shoot end!" <<std::endl;

    return NodeStatus::SUCCESS;
}


void ShootAction::halt() {
    this->_halt_requested.store(true);
    this->stop();
}

void ShootAction::init(ChassisExecutor*& chassis_executor, GimbalExecutor*& gimbal_executor) {
    this->chassis_executor_ = chassis_executor;
    this->gimbal_executor_ = gimbal_executor;
}

void ShootAction::stop(){
    this->gimbal_executor_->Cancel();
    this->chassis_executor_->Cancel();
}

BT::PortsList ShootAction::providedPorts() {
    return {BT::InputPort<uint8_t>("number")};
}