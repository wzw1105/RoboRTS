/*
 * @Author: chen
 * @Date: 2022-03-09 20:12:32
 * @LastEditTime: 2022-05-20 13:48:29
 * @LastEditors: Please set LastEditors
 * @Description: 
 * @FilePath: \roborts_ws\src\RoboRTS\test_pkg\src\Functions.h
 */

#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

#include "share_head.h"
#include "tool/tool.h"

class ComputeShootNumber : public BT::SyncActionNode {
    public:
        ComputeShootNumber(const std::string &name, const BT::NodeConfiguration &config) : 
            BT::SyncActionNode(name, config)
        {
        }
        
        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts();
};

/**
 * ? 行为节点 ActionNode
 * ? 计算攻击敌人的移动目的地
 * ! 选取最佳击打位置
 */
BT::NodeStatus Compute_Fight_MoveGoal();


/**
 * ? 行为节点 ActionNode
 * ? 计算巡逻的移动目的地
 * ! 选取最佳击打位置
 */
BT::NodeStatus Compute_Patrol_MoveGoal();


/**
 * ? 行为节点 ActionNode
 * ? 计算前往加成区的移动目的地
 * ! 选取
 */
BT::NodeStatus Compute_Addition_MoveGoal();

/**
 * @brief 到达目的地后，更新加成区激活状态
 * 
 * @return BT::NodeStatus 
 */
BT::NodeStatus UpdateAdditionActiveState();

/**
 * ? 行为节点 ActionNode
 * ? 计算逃跑的移动目的地
 * ! 选取最佳击打位置
 */
BT::NodeStatus Compute_Escape_MoveGoal();


/**
 * ? 计算撤出加成/惩罚区的目标位置
 *
 */
BT::NodeStatus Compute_Withdraw_Addition_MoveGoal();

/**
 * @brief Set the Goal object
 * * 设置移动目标
 * @return BT::NodeStatus 
 */
BT::NodeStatus SetGoal();

/**
 * @brief Set the Search Back Status object
 * * 设置绕后状态
 * @return BT::NodeStatus 
 */
BT::NodeStatus SetSearchBackStatus();

/**
 * @brief Cancel the Search Back Status object
 * * 取消绕后状态
 * @return BT::NodeStatus 
 */
BT::NodeStatus CancelSearchBackStatus();

/**
 * @brief get bullets addition goal
 * ? 获取子弹加成区目标位姿
 */
BT::NodeStatus GetBulletsAdditionGoal(/* BT::TreeNode &self */ );

/**
 * @brief get HP addition goal
 * ? 获取血量加成区目标位姿
 */
BT::NodeStatus GetHPAdditionGoal(/* BT::TreeNode &self */ );


/**
 * @brief Set the Addition Adjust Goal object
 * ? 根据当前的加成区，确定调整目标
 * @return BT::NodeStatus 
 */
BT::NodeStatus SetAdditionAdjustGoal();


/**
 * @brief 
 * 
 */
BT::NodeStatus SetGameEnd();

/**
 * ? 功能函数
 * ? 计算一次设计多少发子弹
 * ! 得到接口 uint8_t
 */
BT::NodeStatus Compute_ShootNumber(BT::TreeNode &self);

/**
 * ? 功能函数
 * ? 计算射击到敌人的概率
 * ! 得到概率 做判断
 */
double Compute_My_HittingAccuracy_(geometry_msgs::PoseStamped my_pose,
    geometry_msgs::PoseStamped enemy_pose,
    bool if_enemy_armor_detected);



/**
 * ? 功能函数
 * ? 计算被敌人射击的概率
 * ! 得到概率 做判断
 */
double Compute_Enemy_HittingAccuracy_(geometry_msgs::PoseStamped my_pose,
    geometry_msgs::PoseStamped enemy_pose);

/**
 * ? 功能函数
 * ? 计算两个位置之间有没有障碍物
 * ! 得到bool
 *
bool IF_Obstacle_Between(geometry_msgs::PoseStamped my_pose,
    geometry_msgs::PoseStamped dest_pose, CostmapPtr costmap_ptr) {
    //* 获取位置在costmap中的位序
    //* 判断是否存在障碍物
    return false;
}
*/

#endif