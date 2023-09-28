#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "../executor/chassis_executor.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "../blackboard/blackboard.h"
#include "../behavior_tree/behavior_state.h"

namespace roborts_decision{

class Patrol : public BT::CoroActionNode{

public:
	Patrol(const std::string& name ) : BT::CoroActionNode(name,{}) {
		this->patrol_goal_serial = 0;
        _halt_requested.store(false);
	}
	BT::NodeStatus tick() override {

	    BehaviorState executor_state;
        this -> patrol_index = GetPatrolIndex();
        this -> patrol_num = blackboard_ptr_ -> amount_of_patrol_goals[this -> patrol_index];
        this -> patrol_goal_serial = 0;
        this -> last_stop_time = ros::Time::now() - ros::Duration(600);
        this -> last_goal_reached = false;

        while (!this->_halt_requested && ros::ok()) {
            executor_state = this->chassis_executor_->Update();

            if(executor_state == BehaviorState::FAILURE) {
                this->patrol_goal_serial = (this -> patrol_goal_serial + 1) % (this -> patrol_num); // go next
            }
            else if(executor_state == BehaviorState::SUCCESS) {
                this->patrol_goal_serial = (this -> patrol_goal_serial + 1) % (this -> patrol_num);
                last_goal_reached = true;
                last_stop_time = ros::Time::now();
            }
            if(last_goal_reached) {
                if(ros::Time::now().toSec() - last_stop_time.toSec() < 0.7) setStatusRunningAndYield();
                else last_goal_reached = false;
            }

            this->patrol_goal = blackboard_ptr_ -> default_patrol_goal_array[this -> patrol_index][this->patrol_goal_serial];
            if ( executor_state != BehaviorState::RUNNING ) {
                std::cout << "Patrol Goal : " <<  this->patrol_goal.pose.position.x << " " <<  this->patrol_goal.pose.position.y << "\n";
                this -> chassis_executor_ -> Execute(this->patrol_goal);
            }

            setStatusRunningAndYield();
        }
        
        return _halt_requested || executor_state == BehaviorState::FAILURE ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }
    void halt() override {
        this->patrol_goal_serial = 0;
        this->chassis_executor_->Cancel();
        this->_halt_requested.store(true);
        
        //* don't forget to call this function
        CoroActionNode::halt();
    }

    void init(std::shared_ptr<BlackBoard>  blackboard, ChassisExecutor* chass) {
        blackboard_ptr_ = blackboard;
        chassis_executor_ = chass;
    }

private:
	std::atomic_bool _halt_requested;
	ChassisExecutor* chassis_executor_;
    uint8_t patrol_index = 0;
	uint8_t patrol_goal_serial;	//* 区域序号，按顺序前往目的地巡逻
    uint8_t patrol_num = 4;
    ros::Time last_stop_time;
    bool last_goal_reached;

	geometry_msgs::PoseStamped patrol_goal;
    std::shared_ptr<BlackBoard>  blackboard_ptr_;


    int GetPatrolIndex() {
        return blackboard_ptr_ -> friend_dead ? 2 : this -> blackboard_ptr_ -> self_id;
    }
};

}