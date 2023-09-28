/*
 * @Author: kidding
 * @Date: 2022-03-09 20:12:32
 * @LastEditTime: 2022-05-21 22:45:30
 * @LastEditors: Please set LastEditors
 * @Description: 
 * @FilePath: \roborts_ws\src\RoboRTS\test_pkg\src\PreConditions.h
 */
#ifndef _PRECONDITIONS_H_
#define _PRECONDITIONS_H_

#include "enum_class.h"
#include "./functions/Functions.h"

/**
 * @brief 比赛前五秒，开启摩擦轮，
 * 
 * @return BT::NodeStatus 
 */
BT::NodeStatus Awake_Condition() {
    if (blackboard_ptr->game_status == GameStatus::Five_Second_CD) {
        return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::FAILURE;
}

/**
 * @brief not game status
 */
BT::NodeStatus NotGameStage() {
    if (blackboard_ptr->game_status == GameStatus::Five_Second_CD || blackboard_ptr->game_status == GameStatus::End)
        return BT::NodeStatus::FAILURE;
    if (blackboard_ptr->game_status != GameStatus::Game)
        return BT::NodeStatus::SUCCESS;
    
    return BT::NodeStatus::FAILURE;
}

/**
 * @brief 比赛结束
 * 
 * @return BT::NodeStatus 
 */
BT::NodeStatus GameEnd() {
    if (blackboard_ptr->game_status == GameStatus::End)
        return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::FAILURE;
}


/**
 * @brief 加成区即将更新
 * ? 检查加成区是否快更新
 * @return BT::NodeStatus 
 */
BT::NodeStatus CheckAdditionGoingRefresh() {
    if (blackboard_ptr->game_status != GameStatus::Game)
    {
        if ((blackboard_ptr->game_remaining_time <= 123
                && blackboard_ptr->game_remaining_time > 120)
                || (blackboard_ptr->game_remaining_time <= 63
                && blackboard_ptr->game_remaining_time > 60)
            ) 
        {
            return BT::NodeStatus::SUCCESS;
        }
    }
    
    return BT::NodeStatus::FAILURE;
}
/**
 * @brief 加成区即将更新，检查当前位置是否安全
 * ? 如果当前位置处于留个区域内，需要及时离开加成区
 * ! 加成区刷新前三秒开启
 */
BT::NodeStatus CheckPoseInAddition() {
    //* 继续判断是否处于六个加成惩罚区域内
    for (int i = 0; i < 6; i++)
    {
        if (
            Compute_Distance_BetweenBoth(blackboard_ptr->my_Pose.pose.position.x,
                                         blackboard_ptr->my_Pose.pose.position.y,
                                         blackboard_ptr->F_1_6[i].pose.position.x,
                                         blackboard_ptr->F_1_6[i].pose.position.y) <= 0.5)
        {
            blackboard_ptr->relative_addition_area_number = i + 1;
            return BT::NodeStatus::SUCCESS;
        }
    }
    blackboard_ptr->relative_addition_area_number = 0;
    return BT::NodeStatus::FAILURE;    
}

/**
 * @brief 前往子弹加成区
 * ? 条件满足时，前往子弹加成区
 */
BT::NodeStatus BulletsAddition_Condition() {
    if (blackboard_ptr->game_status != GameStatus::Game)
        return BT::NodeStatus::FAILURE;
    
    if (!blackboard_ptr->if_Bullets_Supply_Active) {
        //* 子弹足够多、血量还行、检测到敌方装甲板
        if ((blackboard_ptr->my_bullets > 100 || blackboard_ptr->my_robot_HP < 200) && blackboard_ptr->if_enemy_armor_detected) {
            return BT::NodeStatus::FAILURE;
        }
        if ( blackboard_ptr->if_being_attacked || !blackboard_ptr->chassis_enable) 
        {
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

/**
 * @brief 前往血量加成区
 *  ? 前往血量加成区
 * 
 */
BT::NodeStatus HPAddition_Condition() {
    if (blackboard_ptr->game_status != GameStatus::Game)
        return BT::NodeStatus::FAILURE;

    if (blackboard_ptr->if_being_attacked || !blackboard_ptr->chassis_enable)
    {
        return BT::NodeStatus::FAILURE;
    }

    if (!blackboard_ptr->if_HP_Recovery_Active)
    {
        if (blackboard_ptr->my_robot_HP > 1500)
        {
            return BT::NodeStatus::FAILURE;
        }

        if (blackboard_ptr->if_enemy_detected && blackboard_ptr->my_bullets > 50)
        {
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}


/**
 * ? 前提条件 Precondition
 * ? 攻击敌人的叶子节点的前提条件
 * ! 默认加成区优先级更高
 */
BT::NodeStatus Fight_Condition() {
    if (blackboard_ptr->game_status != GameStatus::Game)
        return BT::NodeStatus::FAILURE;

    if (!blackboard_ptr->if_Bullets_Supply_Active)
        return BT::NodeStatus::FAILURE;

    //* 没有子弹不参与战斗
    if (blackboard_ptr->my_bullets <= 1)
        return BT::NodeStatus::FAILURE;
        
    if (blackboard_ptr->if_being_attacked && blackboard_ptr->robot_damage.damage_source != 0 ) {
        return BT::NodeStatus::FAILURE;
    }
    // //* 正在绕后中ing-- 只有在绕后时刻search_back_status才会为true
    // if (blackboard_ptr->search_back_status)
    // {
    //     return BT::NodeStatus::SUCCESS;
    // }
    //* 检测到敌人
    if (blackboard_ptr->if_enemy_detected) {
        if (blackboard_ptr->my_robot_HP > 200 ) {
            if (blackboard_ptr->if_being_attacked && blackboard_ptr->my_robot_HP < 500)
            {
                return BT::NodeStatus::FAILURE;
            }
            if (blackboard_ptr->enemy_distance > 3.5) 
            {
                return BT::NodeStatus::FAILURE;
            }
            std::cout << "fight condition ok" << std::endl;
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }
    else {
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::FAILURE;
}

/**
 * ? 前提条件 Precondition
 * ? 执行巡逻的叶子节点的前提条件
 * ! 默认加成区、攻击敌人、逃跑的优先级更高
 */
BT::NodeStatus Patrol_Condition() {
    if (blackboard_ptr->game_status != GameStatus::Game)
        return BT::NodeStatus::FAILURE;

    if (blackboard_ptr->game_remaining_time >= 176)
        return BT::NodeStatus::FAILURE;
    if (!blackboard_ptr->if_Bullets_Supply_Active) {
        return BT::NodeStatus::FAILURE;
    }
    if (blackboard_ptr->if_enemy_detected || blackboard_ptr->my_robot_HP < 500 || blackboard_ptr->if_being_attacked)
        return BT::NodeStatus::FAILURE;
    
    return BT::NodeStatus::SUCCESS;
}

/**
 * ? 前提条件 Precondition
 * ? 原地扭摆休息的叶子节点的前提条件
 * ! 默认加成区、攻击敌人、逃跑的优先级更高
 */
BT::NodeStatus Idle_Condition() {

    if (blackboard_ptr->game_status != GameStatus::Game)
    {
        return BT::NodeStatus::FAILURE;
    }
    if (!blackboard_ptr->if_Bullets_Supply_Active || !blackboard_ptr->if_Enemy_HP_Recovery_Active)
    {
        return BT::NodeStatus::FAILURE;
    }
    //* 没子弹---快没血了---有敌人----没有加成区
    if (blackboard_ptr->if_being_attacked && blackboard_ptr->my_bullets <=1 )
    {
        return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::FAILURE;
}


/**
 * ? 前提条件 Precondition
 * ? 受攻击后，转身防卫
 * ! 被攻击即开启防卫
 */
BT::NodeStatus TurnDefend_Condition(){
    if (blackboard_ptr->game_status != GameStatus::Game)
        return BT::NodeStatus::FAILURE;
    if (!blackboard_ptr->if_Bullets_Supply_Active) 
    {
        return BT::NodeStatus::FAILURE;
    }
    if (blackboard_ptr->if_being_attacked && blackboard_ptr->robot_damage.damage_source != 0 )
    {
        return BT::NodeStatus::SUCCESS;
    }
    //* 被攻击 而且有子弹
    if(blackboard_ptr->if_being_attacked && blackboard_ptr->my_bullets >=20) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

/**
 * ? 前提条件 Precondition
 * ? 躲避敌人的叶子节点的前提条件
 * ! 默认加成区、攻击敌人的优先级更高
 */
BT::NodeStatus Escape_Condition() {
    if (blackboard_ptr->game_status != GameStatus::Game)
        return BT::NodeStatus::FAILURE;
    if (blackboard_ptr->if_being_attacked) {    //* 前提--被攻击
        if (blackboard_ptr->my_bullets <= 10 ||      //* 没子弹
            blackboard_ptr->my_robot_HP <= 240 ||   //* 没血
            !blackboard_ptr->if_enemy_detected )    //* 没检测到敌人
            return BT::NodeStatus::SUCCESS;
        else {
            
        }
    }
    return BT::NodeStatus::FAILURE;
}

/**
 * ? 前提条件 Preconditin
 * ? 判断当前位置是否需要移动之后进行射击
 * ! 可以综合历史经验进行判断
 */
BT::NodeStatus Whether_Suitable_To_Fight() {
    //* test 
    if (blackboard_ptr->game_status != GameStatus::Game){
        return BT::NodeStatus::FAILURE;
    }
    // //* 正在绕后中ing-- 只有在绕后时刻search_back_status才会为true
    // if (blackboard_ptr->search_back_status)
    // {
    //     return BT::NodeStatus::SUCCESS;
    // }
    if (blackboard_ptr->enemy_distance >= blackboard_ptr->shoot_distance_limit_ - 0.5) {
        return BT::NodeStatus::FAILURE;
    }
    if (!blackboard_ptr->if_enemy_armor_detected)
    {
        return BT::NodeStatus::FAILURE;
    }
    std::cout << "suitable to fight" << std::endl;

    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus Whether_Armor_Detected()
{
    if (blackboard_ptr->game_status != GameStatus::Game){
        return BT::NodeStatus::FAILURE;
    }
    if (blackboard_ptr->if_enemy_armor_detected)
    {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

/**
 * @brief 判断当前位置是否已在加成区
 * ? 前提条件 Preconditin
 * ! 匹配目的地
 * @return BT::NodeStatus 
 */
BT::NodeStatus Whether_Suitable_Bullets_Addition() {
    //* 非比赛阶段 条件休想成立！
    if (blackboard_ptr->game_status != GameStatus::Game){
        return BT::NodeStatus::FAILURE;
    }
    //done 判断是否到达目的加成区且成功刷取
    //***   刷过子弹加成区之后，判断是否已经刷上  ****
    if(blackboard_ptr->if_Bullets_Supply_Active)
        return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::FAILURE;
}

/**
 * @brief 
 *! Linux 系统设置 ulimit命令设置（-a 查看所有设置，/第一个）
 *! 编译时，加-g参数，SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")
 *! 运行段错误程序，会在当前目录生成文件名为core的文件
 *! 打开gdb（gdb 加文件名）
 *! 在gdb命令行 输入core-file core文件名
 */

/**
 * @brief 血量加成是否已经完成
 * 
 * @return BT::NodeStatus 
 */
BT::NodeStatus Whether_Suitable_HP_Addition() {
    //done *****  ****
    //* 非比赛阶段 条件休想成立！
    if (blackboard_ptr->game_status != GameStatus::Game){
        return BT::NodeStatus::FAILURE;
    }
    //done 判断是否到达目的加成区且成功刷取
    //***   刷过子弹加成区之后，判断是否已经刷上  ****
    if(blackboard_ptr->if_HP_Recovery_Active)
        return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus Whether_Search_Back()
{
    uint8_t accu = 0;
    accu = std::accumulate(blackboard_ptr->enemy_front_pose_history_.begin(), blackboard_ptr->enemy_front_pose_history_.end(), accu);
    if (accu >= blackboard_ptr->enemy_front_pose_history_threshold_)
    {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

/**
 * ? 前提条件 Preconditin
 * ? 判断当前位置是否满足躲避敌人escape要求
 * ! 满足躲避要求
 */
BT::NodeStatus Whether_Suitable_Escape_Location() {
    //todo ''''''判断是否到足以躲避攻击的安全区
    //*******
    return BT::NodeStatus::SUCCESS;
}
/**
 * @brief 是否伤害已达到胜利条件
 * 
 * @return BT::NodeStatus 
 */
BT::NodeStatus Whether_Hitting_Win_Condition(){
    if (blackboard_ptr->my_Hittig - blackboard_ptr->enemy_hitting > 200)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else 
    {
        return BT::NodeStatus::FAILURE;
    }
}

#endif