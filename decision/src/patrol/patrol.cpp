/*
 * @Author: your name
 * @Date: 2022-03-25 16:07:23
 * @LastEditTime: 2022-05-21 15:34:37
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: \roborts_ws\src\RoboRTS\test_pkg\src\skills\patrol.cpp
 */
#include "./patrol.h"
#include "enum_class.h"

using namespace ENUM_CLASS;

//* tick 执行
NodeStatus Patrol::tick() {
	std::cout << "patrol action" << std::endl;
	this->_halt_requested.store(false);
	this->chassis_executor_->Cancel();
	//* 执行器状态 
	BehaviorState executor_state;
	//* 进入循环---未经打断，重复执行巡逻逻辑
	while (!this->_halt_requested && ros::ok())
	{
		//* 不断更新底盘执行器状态
		executor_state = this->chassis_executor_->Update();
		//* 检查退出条件
		if (this->_halt_requested || executor_state == BehaviorState::FAILURE)
		{
			//this->patrol_goal_serial = (this->patrol_goal_serial + 1)% blackboard_ptr->patrol_goals_.size();
			
			break;
		}
		
		//* 成功到达一个目的地
		if (executor_state == BehaviorState::SUCCESS) {
			this->patrol_goal_serial =
				(this->patrol_goal_serial + 1) % blackboard_ptr->patrol_goals_.size();
		}
		//* 执行移动
		if ( executor_state != BehaviorState::RUNNING )
		{
			if (blackboard_ptr->friend_HP <= 0) {
				this->patrol_goal_serial = (this->patrol_goal_serial + 1) % blackboard_ptr->amount_of_search_region_1;
				this->patrol_goal = blackboard_ptr->search_region_1_[this->patrol_goal_serial];
			} else {
				if(blackboard_ptr->self_id == 1) {
					this->patrol_goal_serial = (this->patrol_goal_serial + 1) % blackboard_ptr->amount_of_search_region_1;
					this->patrol_goal = blackboard_ptr->search_region_1_[this->patrol_goal_serial];
				}
				else {
					this->patrol_goal_serial = (this->patrol_goal_serial + 1) % blackboard_ptr->amount_of_search_region_2;
					this->patrol_goal = blackboard_ptr->search_region_2_[this->patrol_goal_serial];
				}
			}
			//* 目的地
			//this->patrol_goal = blackboard_ptr->patrol_goals_[this->patrol_goal_serial];
			this->chassis_executor_->Execute(this->patrol_goal);
		}

		//*  挂起线程，tick()返回running
		setStatusRunningAndYield();
	}
	std::cout << "patrol end" << std::endl;
	//* stop函数，处理任务结束工作
	this->stop();
	return _halt_requested ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
}

Patrol::~Patrol(){}

//* 初始化私有成员---底盘执行器
void Patrol::init(ChassisExecutor*& chassis_executor, GimbalExecutor*& gimbal_executor) {
	this->chassis_executor_ = chassis_executor;
	this->gimbal_executor_ = gimbal_executor;
}

//* halt 终端函数
void Patrol::halt() {
	//* 终止任务
	this->_halt_requested.store(true);
	//stop();
	//* don't forget to call this function
    CoroActionNode::halt();
}

void Patrol::stop(){
	this->patrol_goal_serial = 0;
	this->chassis_executor_->Cancel();	//* 取消底盘控制
}