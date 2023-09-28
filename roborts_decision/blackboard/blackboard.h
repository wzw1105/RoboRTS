#ifndef _BLACKBOARD_H_
#define _BLACKBOARD_H_

#include <memory>
#include <iostream>
#include <ros/ros.h>
#include <cmath>
#include <algorithm>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "io/io.h"

#include <geometry_msgs/PoseStamped.h>
#include <roborts_msgs/GimbalAngle.h>
#include <nav_msgs/Odometry.h>


//* tf 四元数和欧拉角的相互转换
#include <tf/tf.h>
#include <mutex>
#include <tf/transform_datatypes.h>

//! Referee System 
//* 血量信息、比赛阶段、子弹数量等等
#include <roborts_msgs/GameResult.h>
#include <roborts_msgs/GameStatus.h>
#include <roborts_msgs/GameRobotHP.h>
#include <roborts_msgs/GameRobotBullet.h>
#include <roborts_msgs/GameZoneArray.h>

#include <roborts_msgs/RobotStatus.h>
#include <roborts_msgs/RobotHeat.h>
#include <roborts_msgs/RobotDamage.h>
#include <roborts_msgs/RobotShoot.h>
#include <roborts_msgs/LurkStatus.h>
#include "roborts_msgs/FricWhl.h"
#include "roborts_msgs/ShootCmd.h"

//! 装甲板检测头文件
#include <roborts_msgs/ArmorMsgs.h>
#include <roborts_msgs/ArmorMsg.h>

//! 哨岗检测数据
#include <roborts_msgs/CarMsgs.h>
#include <roborts_msgs/CarMsg.h>

//! 代价地图
#include "costmap/costmap_interface.h"
#include "../executor/gimbal_executor.h"

namespace roborts_decision{

const float init_k_ = 0.026;
const float GRAVITY = 9.78;
const double PI = 3.1415926535;

//! 代价地图
typedef roborts_costmap::CostmapInterface CostMap;
typedef roborts_costmap::Costmap2D CostMap2D;

class BlackBoard {
public:
    BlackBoard(GimbalExecutor* gimbal);
    ~BlackBoard(){};

    //# referee system msgs
    int self_color = -1, self_id = -1; //self_color: 0-red 1-blue self_id: 0-car1 1-car2
    uint16_t robot_bullets[2][2]; //GameRobotBullet提供
    uint16_t robot_hp[2][2]; //GameRobotHP提供
    uint16_t self_remain_hp; //RobotStatus提供

    uint8_t game_status = 0; //READY = 0, PREPARATION = 1, INITIALIZE = 2, FIVE_SEC_CD = 3, GAME = 4, END = 5
    uint8_t lurk_status = 0; //lurk_status_normal=0, lurk_status_ready=1, lurk_status_lurking=2
    uint16_t remaining_time = 1000;

    roborts_msgs::RobotDamage robot_damage; //! 损伤扣血部位、装甲板受伤及编号
    bool gimbal_enable = true; //! 云台是否可操控
    bool chassis_enable = true; //! 底盘是否可操控
    bool shooter_enable = true; //! 枪口是否可操控
    uint8_t shoot_frequency; //! 枪口信息-枪口射击频率
    float shoot_speed = 18.0; //! 枪口信息-子弹射速
    uint16_t shooter_heat; //! 枪口热量
    uint16_t heat_cooling_rate; //! 热量冷却速率
    uint16_t heat_cooling_limit; //! 热量冷却限制

    //# 自己的相关信息
    geometry_msgs::PoseStamped default_patrol_goal_array[3][10]; //! positions for patrolling
    uint8_t amount_of_patrol_goals[3] = {4, 6, 8}; //! number of positions for patrolling
    geometry_msgs::PoseStamped self_pose; //! 自身位姿,
    geometry_msgs::PoseStamped boot_area_pose; //! 本车启动区的位姿
    geometry_msgs::PoseStamped my_goal; //! AI机器人运动目标位置
    nav_msgs::Odometry odom_info_; //! 里程计信息

    //# 小伙伴的信息
    geometry_msgs::PoseStamped friend_Pose; //! 友军位姿
    ros::Time friend_pose_time; //友军位姿时间戳
    bool friend_dead = false; //友军死亡

    //# 加成区
    geometry_msgs::PoseStamped F_1_6[6]; //! F1 ~ F6 区域的位姿，以红方原点为坐标系
    bool bullets_add_area_active = false; //! 子弹加成区是否存在
    int bullets_add_position_id; // index for F_1_6
    bool blood_add_area_active = false; //血量加成区
    int blood_add_position_id;
    roborts_msgs::GameZoneArray game_zone_array; //RED_HP_RECOVERY=1, RED_BULLET_SUPPLY=2, BLUE_HP_RECOVERY=3, BLUE_BULLET_SUPPLY=4, DISABLE_SHOOTING=5, DISABLE_MOVEMENT=6

    //# 哨岗信息
    roborts_msgs::CarMsgs guard_msgs;

    //# 记录状态
    bool init_data_with_robot_id = false;
    ros::Time recent_armor_damage_time; //记录最近一次被打时间
    uint8_t recent_armor_damege_id;
    
    //## 射击相关
    int total_index = 100; //一个周期对应的回调函数数量上限
    roborts_msgs::ArmorMsg cur_hit_armor; //记录当前选取的要打击的装甲板
    roborts_msgs::ArmorMsg choosed_armor[100]; //一个周期内每次选取的装甲板, 10是考虑到最低检测帧率为10，故一个周期时间小于等于1s
    bool choose_armor[100]; //每一个index对应的时间是否选取装甲板
    ros::Time cycle_enermy_detect_time[100]; //记录周期内的每一个时间
    geometry_msgs::PoseStamped cycle_enemy_detect_map_position[100]; //记录一个周期内检测到的装甲板的map坐标系位置
    int cur_armor_index = 0; //当前时间所在的index
    int shoot_num = 0; //当前决策的子弹打击数量
    ros::Time recent_enemy_detect_time; //记录最近一次检测到敌人的时间
    bool last_shoot_enable = true; 
    ros::ServiceClient fric_client;
    ros::ServiceClient shoot_client;
    bool detected_enermy = false;
    std::mutex shoot_mtx_;
    bool this_frame_detected = false;
    float cur_pitch_ = 0;
    float yaw_change_threshold = 0.01;
    float pitch_change_threshold = 0.01;
    const double yaw_change_unit_ = 0.05;
    float init_k_ = 0.026;
    float GRAVITY = 9.78;
    double PI = 3.1415926535;
    float h = 0.2;
    

    //## 记录自己血量信息
    int total_hp_index = 100;
    uint16_t self_recent_hp[100];
    int cur_hp_index = 0;

    //移动目标
    geometry_msgs::PoseStamped enemy_area_goal;

private:
    //data
    std::shared_ptr<tf::TransformListener> tf_ptr_;
    std::shared_ptr<CostMap> costmap_ptr_;
    ros::NodeHandle nh;

    //玻璃信息
    int grass_obstacle_info[4][4] = {
        {142, 20, 21, 5},
        {128, 70, 5, 20},
        {30, 1, 5, 21},
        {1, 66, 20, 5}
    };

    //variables
    float min_area_for_resample = 800; //
    float max_dis_for_shoot = 3.5; //射击的最大距离
    float no_move_pos_threshold = 0.2; //1s内位置变化小于这个阈值视为不动，目标是打击静止敌方。
    double lurk_status_danger_shoot_distance = 2; //ID与自己不同的情况下装甲板和队友之间的距离小于这个值不会打击
    int bullet_min_cell_dis = 1;

    //executor
    GimbalExecutor* gimbal_executor_;
    
    //Subscriber
    ros::Subscriber ros_sub_referee_HP_;        //! 血量 订阅
    ros::Subscriber ros_sub_referee_Bullets_;   //! 子弹数量 订阅
    ros::Subscriber ros_sub_referee_GameZoneArray_; //! 加成/惩罚区域 订阅
    ros::Subscriber ros_sub_referee_GameStatus_;    //! 比赛阶段 订阅
    ros::Subscriber ros_sub_referee_LurkStatus_;    //! 潜伏状态 订阅
    ros::Subscriber ros_sub_referee_Shoot_info_;    //! 机器人自身枪口信息 订阅
    ros::Subscriber ros_sub_referee_Robot_status_;  //! 机器人状态信息：血量、冷却、地盘/云台/射击enable状态 订阅
    ros::Subscriber ros_sub_referee_Robot_damage_;  //! 机器人损伤扣血情况，被击打的装甲及编号 订阅
    ros::Subscriber ros_sub_referee_Robot_heat_;    //! power and heat data 热量信息 订阅
    ros::Subscriber ros_sub_robot_pose_;    //! 定位信息
    ros::Subscriber ros_sub_robot_detect_;  //! 识别敌方信息
    ros::Subscriber ros_sub_robot_friendpose_;  //! 友军
    ros::Subscriber ros_sub_robot_odom_;    //! 自身里程计odom信息
    ros::Subscriber ros_sub_guard_msg_; // guard msgs

    //callback
    void Sub_GameRobotHP_CallBack_(const roborts_msgs::GameRobotHP::ConstPtr& game_robot_hp); //* 血量订阅的回调函数
    void Sub_GameRobotBullet_CallBack_(const roborts_msgs::GameRobotBullet::ConstPtr& game_robot_bullet); //* 子弹数量订阅的回调函数
    void Sub_GameZoneArrayStatus_CallBack_(const roborts_msgs::GameZoneArray::ConstPtr& game_zone_array_status); //* 加成/惩罚区域数组 订阅回调函数
    void Sub_GameStatus_CallBack_(const roborts_msgs::GameStatus::ConstPtr& game_status_);//* 比赛阶段订阅回调函数
    void Sub_GameLurkStatus_CallBack_(const roborts_msgs::LurkStatus::ConstPtr& lurk_status_);//* 潜伏机制订阅回调函数
    void Sub_RobotStatus_CallBack_(const roborts_msgs::RobotStatus::ConstPtr& robot_status);//* 机器人自身状态订阅回调函数
    void Sub_RobotDamage_CallBack_(const roborts_msgs::RobotDamage::ConstPtr& robot_damage);//* 机器人损伤扣血情况：装甲板被击打及编号、超限等订阅回调函数
    void Sub_RobotHeat_CallBack_(const roborts_msgs::RobotHeat::ConstPtr& robot_heat);//* 机器人热量信息订阅回调函数
    void Sub_RobotShootInfo_CallBack_(const roborts_msgs::RobotShoot::ConstPtr& robot_shoot);//* 机器人枪口信息订阅回调函数
    void Sub_RobotPose_CallBack_(const geometry_msgs::PoseStamped::ConstPtr& robot_pose);//* 回调函数--机器人自身位置信息的回调函数
    void Sub_RobotDetectEnemy_CallBack_(const roborts_msgs::ArmorMsgs::ConstPtr& msgs);//* 回调函数--机器人识别到敌人信息的回调函数
    void Sub_Robot_FriendPose_CallBack_(const geometry_msgs::PoseStamped::ConstPtr& robot_pose);//* 回调函数--机器人友军威姿订阅回调函数
    void Sub_Robot_Odom_CallBack_(const nav_msgs::Odometry::ConstPtr &odom_info);//* 回调函数--机器人自身的odom里程计消息订阅回调函数
    void Sub_Guard_Msg_CallBack_(const roborts_msgs::CarMsgs::ConstPtr& guard_msgs);
    
    void PublishPitchYawMsgs(float &next_yaw, float &next_pitch);
    float GetPitch(float x, float y, float v);
    float BulletModel(float x, float v, float angle);
    //functions
    double GetNextY(double x, double y, double k); //辅助
    bool CheckVerticalLineCross(int x, int y_1, int y_2); //辅助
    bool CheckCrossWithHighObstacle(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped end); //检查是否穿过玻璃
    bool CheckArmorCanShoot(roborts_msgs::ArmorMsg &armor, geometry_msgs::PoseStamped &armor_pose); //根据比赛阶段的不同（前两分钟和后一分钟）检查该装甲板是否可以打击（依据颜色和ID和友军位姿进行判断）
    bool ReFindOptimalArmorForShoot(std::vector<roborts_msgs::ArmorMsg> &msgs, roborts_msgs::ArmorMsg &decision_armor, ros::Time &now_); //在所有装甲板里面重新选取一个最优的装甲板
    bool ChooseArmorForShoot(std::vector<roborts_msgs::ArmorMsg> msgs, roborts_msgs::ArmorMsg &decision_armor, int &shoot_num); //正常状态下选取打击的装甲板
    bool GetArmorMapPose(geometry_msgs::Point &armor_pose_, geometry_msgs::PoseStamped &global_pose);  //获得装甲板在map坐标系下的坐标
    bool GetArmorBaseLinkPose(geometry_msgs::Point &armor_pose_, geometry_msgs::PoseStamped &global_pose)
    double Distance(geometry_msgs::PoseStamped &p_1, geometry_msgs::PoseStamped &p_2); //计算两点距离
    double CalculateArmorScore(roborts_msgs::ArmorMsg &aomor);

};

}


#endif