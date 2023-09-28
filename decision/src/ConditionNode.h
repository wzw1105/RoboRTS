#include <ros/ros.h>
#include "share_head.h"

namespace RTS_DECISION{

class MyConditionNode{
public:
    MyConditionNode(std::shared_ptr<BlackBoard> blackboard) : blackboard_ptr_(blackboard) {};
    ~MyConditionNode() {};

    // 开始前5秒
    BT::NodeStatus PreParation5Seconds() {
        BT::NodeStatus result = blackboard_ptr_ -> game_status == GameStatus::Five_Second_CD ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        //if(result == BT::NodeStatus::SUCCESS) ROS_INFO("[ Stage : 5 Second CD]");
        return result;
    }

    // 比赛正常阶段
    BT::NodeStatus InGame() {
        BT::NodeStatus result = (blackboard_ptr_ -> game_status == GameStatus::Game) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        //if(result == BT::NodeStatus::SUCCESS) ROS_INFO("[ Stage : In Game]");
        return result;
    }

    //比赛结束
    BT::NodeStatus GameEnd() {
        BT::NodeStatus result = blackboard_ptr_ -> game_status == GameStatus::End ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        //if(result == BT::NodeStatus::SUCCESS) ROS_INFO("[ Stage : GameEnd]");
        return result;
    }

    //足够的血量
    BT::NodeStatus EnoughBlood() {
        BT::NodeStatus result = blackboard_ptr_ ->my_robot_HP > 500 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        //if(result == BT::NodeStatus::SUCCESS) ROS_INFO("[ Blood : Enough]");
        //else ROS_INFO("[ Blood : Not Enough]");
        return result;
    }

    //血量极少
    BT::NodeStatus EmergencyBlood() {
        BT::NodeStatus result = blackboard_ptr_ -> my_robot_HP <= 200 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        //if(result == BT::NodeStatus::SUCCESS) ROS_INFO("[ Blood : Emergency]");
        //else ROS_INFO("[ Blood : Not Emergency]");
        return result;
    }

    //足够的子弹
    BT::NodeStatus EnoughBullets() {
        if(blackboard_ptr_ -> self_color == -1) return BT::NodeStatus::FAILURE;

        BT::NodeStatus result = blackboard_ptr_ -> robot_bullets[blackboard_ptr_ -> self_color][blackboard_ptr_ -> self_id] >= 50 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        //if(result == BT::NodeStatus::SUCCESS) ROS_INFO("[ Bullets : Enough]");
        //else ROS_INFO("[ Bullets : Not Enough] ");
        return result;
    }

    //还有子弹
    BT::NodeStatus HaveBullets() {
        BT::NodeStatus result = blackboard_ptr_ -> robot_bullets[blackboard_ptr_ -> self_color][blackboard_ptr_ -> self_id] > 0 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        //if(result == BT::NodeStatus::SUCCESS) ROS_INFO("[ Bullets : Have]");
        //else ROS_INFO("[ Bullets : Dont Have] ");
        return blackboard_ptr_ -> my_robot_HP >= 50 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;  
    }

    // //检查自己是否正在急速掉血
    // BT::NodeStatus SufferFastHit() {
    //     int blood_lost = int(blackboard_ptr_ -> self_recent_hp[blackboard_ptr_ -> cur_hp_index]) - int(blackboard_ptr_ -> self_recent_hp[(blackboard_ptr_ -> cur_hp_index + 1) % blackboard_ptr_ -> total_hp_index]);
    //     //if(blood_lost > 150) ROS_INFO("[SufferFastHit: Lose %d bloods]", blood_lost);
    //     return blood_lost > 150 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    // }

    // BT::NodeStatus SufferMediumHit() {
    //     int blood_lost = int(blackboard_ptr_ -> self_recent_hp[blackboard_ptr_ -> cur_hp_index]) - int(blackboard_ptr_ -> self_recent_hp[(blackboard_ptr_ -> cur_hp_index + 1) % blackboard_ptr_ -> total_hp_index]);
    //     //if(blood_lost > 80) ROS_INFO("[SufferMediumOrFastHit: Lose %d bloods]", blood_lost);
    //     return blood_lost > 80 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    // }

    //检查最近1秒内是否有可用的哨岗信息???
    BT::NodeStatus GuardMsg1SecondUseful() {
        ros::Time now_ = ros::Time::now();
        BT::NodeStatus result = now_.toSec() - blackboard_ptr_->guard_cars_msg_.stamp_guard.toSec() <= 3.0 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        if(result == BT::NodeStatus::SUCCESS) ROS_INFO("[ Guard : Guard msg In 1 Second Useful ]");
        else ROS_INFO("[ Guard : No Guard msg In 1 Second Useful ]");
        return result;
    }

    // //检查近一秒是否有可打击的敌人
    // BT::NodeStatus DetectedEnemyIn1Second() {
    //     ros::Time now_ = ros::Time::now();
    //     BT::NodeStatus result = BT::NodeStatus::FAILURE;

    //     int detected_num = 0, all_frame_in_1_second = 0;
    //     for(int i = 1; i < blackboard_ptr_ -> total_index; i++) {
    //         int pre_index = (blackboard_ptr_ -> cur_armor_index - i + blackboard_ptr_ -> total_index) % blackboard_ptr_ -> total_index; //对应的循环数组下标
    //         if(now_.toSec() - blackboard_ptr_ -> cycle_enermy_detect_time[pre_index].toSec() > 1.0) break; //只考虑1s内
    //         all_frame_in_1_second += 1;
    //         if(blackboard_ptr_ -> choose_armor[pre_index]) {
    //             detected_num += 1;
    //         }
    //     }
    //     if(all_frame_in_1_second > 0 && detected_num >= (int)(1.0 * all_frame_in_1_second / 3)) result = BT::NodeStatus::SUCCESS;
    //     //if(result == BT::NodeStatus::SUCCESS) ROS_INFO("[ Hittable Enemy : Detected In 1 Second]");
    //     //else ROS_INFO(" [No Hittable Enemy] ");
    //     return result;
    // }

    // //子弹加成区有效
    // BT::NodeStatus BulletsAddAreaAvaliable() {
    //     BT::NodeStatus result = blackboard_ptr_ -> bullets_add_area_active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    //     //if(result == BT::NodeStatus::SUCCESS) ROS_INFO("[ Bullets Add Area Available ]");
    //     return result;
    // }

    // //血量加成区有效
    // BT::NodeStatus BloodAddAreaAvailable() {
    //     BT::NodeStatus result = blackboard_ptr_ -> blood_add_area_active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    //     //if(result == BT::NodeStatus::SUCCESS) ROS_INFO("[ Blood Add Area Available ]");
    //     return result; 
    // }

private:
  std::shared_ptr<BlackBoard>  blackboard_ptr_;
};

}