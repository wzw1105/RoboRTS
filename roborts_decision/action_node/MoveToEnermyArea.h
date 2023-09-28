#ifndef MOVE_TO_ENERMY_AREA_H
#define MOVE_TO_ENERMY_AREA_H

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "../behavior_tree/behavior_state.h"

namespace roborts_decision{

class MoveToEnermyArea : public BT::CoroActionNode{
public:
	MoveToEnermyArea(const std::string& name) : CoroActionNode(name,{}) {
        _halt_requested.store(false);
	}

    BT::NodeStatus tick() override{
        this->chassis_executor_->Cancel();
        BehaviorState executor_state;
        geometry_msgs::PoseStamped Enermy_Area = this -> blackboard_ptr_ -> enemy_area_goal;
        std::cout << "Move To EnermyArea: " <<  Enermy_Area.pose.position.x << " "  << Enermy_Area.pose.position.y << "\n";

        this -> chassis_executor_ -> Execute(this -> blackboard_ptr_ -> enemy_area_goal);
        
        while (!_halt_requested && ros::ok() ) {
            executor_state = this->chassis_executor_->Update();
            if (executor_state ==BehaviorState::SUCCESS || executor_state == BehaviorState::FAILURE) break;
            setStatusRunningAndYield();
        }
        this->chassis_executor_->Cancel();
        //halt或全局规划器执行状态为失败
        return _halt_requested || executor_state == BehaviorState::FAILURE ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }

    void halt() override{
        this->chassis_executor_->Cancel();
        this->_halt_requested.store(true);
        BT::CoroActionNode::halt();
    }

    void init(std::shared_ptr<BlackBoard> blackboard, ChassisExecutor* chass) {
        blackboard_ptr_ = blackboard;
        chassis_executor_ = chass;
    }

private:
	std::atomic_bool _halt_requested;
	ChassisExecutor* chassis_executor_;
    std::shared_ptr<BlackBoard>  blackboard_ptr_;
};


}

#endif