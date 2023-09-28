/*
 * @Author: your name
 * @Date: 2022-03-09 20:12:32
 * @LastEditTime: 2022-05-19 16:06:40
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: 
 * @FilePath: \roborts_ws\src\RoboRTS\test_pkg\src\skills\move_goal.cpp
 */
#include "./move_goal.h"
#include "enum_class.h"

using namespace ENUM_CLASS;


MoveGoal::~MoveGoal(){
    this->chassis_executor_->Cancel();
};

NodeStatus MoveGoal::tick() {
    std::cout << "move goal" << std::endl;
    //* 进入tick() 默认重新开启，退出标志设置为false
    _halt_requested.store(false);
    
    //* chassis执行器 取消操作 置空
    this->chassis_executor_->Cancel();

    ENUM_CLASS::BehaviorState executor_state;

    this->goal_position_ = blackboard_ptr->my_goal;

    while (!_halt_requested && ros::ok() ) {
        if (this->_halt_requested) 
            break;
        //* 执行移动
        executor_state = this->chassis_executor_->Update();
        if ( executor_state == BehaviorState::SUCCESS  
            || executor_state == BehaviorState::FAILURE
            ) {
            break;
        }
        if ( executor_state != ENUM_CLASS::BehaviorState::RUNNING)
        {
            this->chassis_executor_->Execute(this->goal_position_);
        }
        //* 挂起线程，tick()返回running
        setStatusRunningAndYield();
    }
    
    std::cout << "move action end" << std::endl;

    this->chassis_executor_->Cancel();

    return ( (executor_state == BehaviorState::SUCCESS ) ? NodeStatus::SUCCESS : NodeStatus::FAILURE );
}

/**
 * * 停机函数
 */
void MoveGoal::halt() {
    //this->chassis_executor_->Cancel();
    this->_halt_requested.store(true);
    
    //* don't forget to call this function
    CoroActionNode::halt();
}

/**
 * * 初始化变量
 */
void MoveGoal::init(ChassisExecutor* &chassis_executor,GimbalExecutor *gimbal_executor){
    this->chassis_executor_ = chassis_executor;
    this->gimbal_executor_ = gimbal_executor;
}
