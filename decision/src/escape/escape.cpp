/*
 * @Author: your name
 * @Date: 2022-04-01 17:47:27
 * @LastEditTime: 2022-05-06 11:39:39
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置
 * @FilePath: \roborts_ws\src\test_pkg\src\skills\escape.cpp
 */
#include "./escape.h"


NodeStatus Escape::tick() {

    ENUM_CLASS::BehaviorState executor_state;

    while (!_halt_requested && ros::ok() ) {
        if (this->_halt_requested) 
            break;
        //* 执行移动
        executor_state = this->chassis_executor_->Update();
        roborts_msgs::GimbalAngle gimbal_angle;
        gimbal_angle.pitch_angle = blackboard_ptr->gimbal_pitch_offset;
        gimbal_angle.pitch_mode = false;
        gimbal_angle.yaw_mode = true;
        gimbal_angle.yaw_angle = blackboard_ptr->gimbal_yaw_offset;
        gimbal_executor_->Execute(gimbal_angle);

        setStatusRunningAndYield();
    }

    return _halt_requested ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
}

void Escape::halt() {
    this->chassis_executor_->Cancel();
    this->_halt_requested.store(true);
    
    //* don't forget to call this function
    CoroActionNode::halt();
}

void Escape::init(ChassisExecutor* &chassis_executor, GimbalExecutor* &gimbal_executor){
    this->chassis_executor_ = chassis_executor;
    this->gimbal_executor_ = gimbal_executor;
}
