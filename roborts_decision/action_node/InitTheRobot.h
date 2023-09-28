#ifndef INIT_THE_ROBOT_H
#define INIT_THE_ROBOT_H

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "../executor/chassis_executor.h"
#include "../executor/gimbal_executor.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "roborts_msgs/FricWhl.h"
#include "roborts_msgs/GimbalAngle.h"
#include "roborts_msgs/ShootCmd.h"

#include "../behavior_tree/behavior_state.h"

namespace roborts_decision{

class InitTheRobot : public BT::SyncActionNode{
public:
	InitTheRobot(const std::string& name) : SyncActionNode(name,{}) {}

    BT::NodeStatus tick() override{
        if(inited_robot) return BT::NodeStatus::SUCCESS;
        inited_robot = true;

        //open fric
        fricCtrl.request.open = true;
        if(blackboard_ptr_ -> fric_client.call(fricCtrl)) {
            ROS_INFO("Open Fric successfully.");
        }else{
            ROS_INFO("Open Failed.");
        }

        //shoot stop
        shootSrv.request.mode = 0;
        shootSrv.request.number = 0;
        if(!blackboard_ptr_ -> shoot_client.call(shootSrv)) ROS_WARN ("Erros occured, can't shoot!");
        else ROS_INFO("Stop shoot successfully!!!");

        //init gimbal angle

        chassis_executor_ -> Cancel();
        gimbal_executor_ -> Cancel();

        return BT::NodeStatus::SUCCESS;
    }

    void init(std::shared_ptr<BlackBoard>  blackboard, ChassisExecutor* chass, GimbalExecutor* gimbal) {
        blackboard_ptr_ = blackboard;
        chassis_executor_ = chass;
        gimbal_executor_ = gimbal;

    }

private:
    ros::NodeHandle nh;

    //shoot
    roborts_msgs::FricWhl fricCtrl;
    roborts_msgs::ShootCmd shootSrv;

    bool inited_robot = false;
	ChassisExecutor* chassis_executor_;
    GimbalExecutor* gimbal_executor_;
    std::shared_ptr<BlackBoard>  blackboard_ptr_;
};

}

#endif