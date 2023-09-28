#ifndef SWING360_H
#define SWING360_H

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "roborts_msgs/TwistAccel.h"
#include "geometry_msgs/Twist.h"
#include "../executor/chassis_executor.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "../blackboard/blackboard.h"
#include "../behavior_tree/behavior_state.h"

namespace roborts_decision{

class Swing360 : public BT::CoroActionNode{
public:
	Swing360(const std::string& name) : BT::CoroActionNode(name,{}){
        rotate_twist_accel.twist.linear.x = rotate_twist_accel.twist.linear.y = rotate_twist_accel.twist.linear.z = 0;

        rotate_twist_accel.twist.angular.x = rotate_twist_accel.twist.angular.y = 0;
        rotate_twist_accel.twist.angular.z = 1;

        _halt_requested.store(false);
	}
    BT::NodeStatus tick() override{
        this->chassis_executor_->Cancel();
        BehaviorState executor_state;
        
        
        while (!_halt_requested && ros::ok()) {
            executor_state = this->chassis_executor_->Update();
            if(executor_state == BehaviorState::SUCCESS || executor_state == BehaviorState::FAILURE) break;
            this -> chassis_executor_ -> Execute(rotate_twist_accel);
            setStatusRunningAndYield();
        }
        this->chassis_executor_->Cancel();
        //halt或全局规划器执行状态为失败
        return _halt_requested || executor_state ==BehaviorState::FAILURE ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }
    

    void halt() override{
        this->chassis_executor_->Cancel();
        this->_halt_requested.store(true);
        BT::CoroActionNode::halt();
    }
    void init(std::shared_ptr<BlackBoard>  blackboard, ChassisExecutor* chass) {
        blackboard_ptr_ = blackboard;
        chassis_executor_ = chass;
    }



private:
    roborts_msgs::TwistAccel rotate_twist_accel;
	std::atomic_bool _halt_requested;
	ChassisExecutor* chassis_executor_;
    std::shared_ptr<BlackBoard>  blackboard_ptr_;
};

}

#endif