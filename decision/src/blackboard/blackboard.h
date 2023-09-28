#ifndef _BLACKBOARD_H_
#define _BLACKBOARD_H_

#include <iostream>
#include <deque>    //双端vector
#include <numeric>  //accumulate
#include <behaviortree_cpp_v3/bt_factory.h>
//* tf 四元数和欧拉角的相互转换
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include "io/io.h"
#include "enum_class.h"

//** pid
#include "pid.h"
#include "kalman_filter.h"

#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseStamped.h"
#include "roborts_msgs/GimbalAngle.h"
#include "roborts_msgs/ShootCmd.h"
#include "roborts_msgs/ShootControl.h"

//! Referee System 
//* 血量信息、比赛阶段、子弹数量等等
#include "roborts_msgs/GameResult.h"
#include "roborts_msgs/GameStatus.h"
#include "roborts_msgs/GameRobotHP.h"
#include "roborts_msgs/GameRobotBullet.h"
#include "roborts_msgs/GameZoneArray.h"

#include "roborts_msgs/RobotStatus.h"
#include "roborts_msgs/RobotHeat.h"
#include "roborts_msgs/RobotDamage.h"
#include "roborts_msgs/RobotShoot.h"
#include "roborts_msgs/LurkStatus.h"
#include "roborts_msgs/FricWhl.h"

//! 装甲板检测头文件
#include "roborts_msgs/ArmorMsgs.h"
#include "roborts_msgs/ArmorMsg.h"
//! 哨岗消息头文件
#include "roborts_msgs/CarMsg.h"
#include "roborts_msgs/CarMsgs.h"

//! 代价地图
#include "costmap/costmap_interface.h"

//! 调整云台
#include "./gimbal_control.h"

#include "../proto/decision.pb.h"

#include "../executor/gimbal_executor.h"
#include "../executor/chassis_executor.h"

const float init_k_ = 0.026;
//const float GRAVITY = 9.78;
//const double PI = 3.1415926535;


//! 代价地图
typedef roborts_costmap::CostmapInterface CostMap;
typedef roborts_costmap::Costmap2D CostMap2D;

using namespace ENUM_CLASS;
using namespace RTS_DECISION;

enum EnemyColor
{
    BLUE = 0,
    RED = 1
};

class EnemyInfo{
  public:
    EnemyInfo(uint16_t bullets_);
    ~EnemyInfo();
    uint16_t HP;       //* 敌人血量
    uint16_t bullets;     //* 剩余子弹数量
    bool if_detected;     //* 是否被检测到
    bool if_armor_detected; //* 装甲板是否被检测到
    geometry_msgs::PoseStamped enemy_pose;  //* 敌人位姿
    geometry_msgs::Point enemy_camera_pose; //* 敌人装甲板在相机世界的位姿
    double distance;
    uint16_t hit_me_times;  //* 攻击我的次数
    bool if_main_attacker;  //* 是否为敌方主攻
    int main_attacker_coefficient;  //* 攻击系数[危险性]
    bool if_has_pose_info;
};


/**
 * ! blackboard类实现
 * *负责实现数据交互
 */
class BlackBoard {
public:
    typedef std::shared_ptr<BlackBoard> Ptr;

    BlackBoard(ChassisExecutor*& chassis_executor, GimbalExecutor*& gimbal_executor);
    ~BlackBoard();

    bool game_end_;

    //!!!!!!! 我方伤害量
    uint16_t my_Hittig = 0;
    //!!!!!! 敌方伤害量
    uint16_t enemy_hitting = 0;

    //* PID 控制器
    pid_type_def yaw_pid;
    //* PID 参数
    float pid_kp;
    float pid_ki;
    float pid_kd;
    float max_out = 0.785;
    float max_iout = 0.785;
    bool use_pid = false;

    // done ------------ 配置相关 --------------
    //* 策略相关 的 参数文件路径
    std::string param_file_path;

    //! tf
    //? tf 变换 的监听者
    std::shared_ptr<tf::TransformListener> tf_ptr_;

    //? 代价地图
    std::shared_ptr<CostMap> costmap_ptr_;
    CostMap2D* costmap_2d_;
    unsigned char* charmap_;

    //? 云台调整数据互斥锁
    std::mutex mutex_gimbal_angle_;

    // done ---------- 代价地图 ----------
    //* 代价地图引用
    const std::shared_ptr<CostMap> GetCostMap();
    //* 2D 代价地图
    const CostMap2D* GetCostMap2D();
    //* charmap
    const unsigned char* GetCharMap();


    //* 获取敌方机器人死亡情况
    ROBORTS_DEAD getEnemyRobortsDead();
    //* 获取本机器人的启动区位姿
    geometry_msgs::PoseStamped getMyBootPose();
    //* 原地休息时扭摆的模式
    SWING_STATUS ComputeSwingStatus();

    /**
     * @brief Get the Gimbal Yaw Angle object
     * * 获取云台相对于底盘的角度
     * @return float 
     */
    float getGimbalYawAngle();

    // done-------------- 比赛相关  --------------
    //! 潜伏期状态
    LurkingStatus lurk_mode;
    //! 比赛阶段
    GameStatus game_status;
    //! 比赛剩余时间
    uint16_t game_remaining_time;
    //! F1 ~ F6 区域的位姿
    geometry_msgs::PoseStamped F_1_6[6];
    //! 首选几个寻敌区域
    std::vector<geometry_msgs::PoseStamped> patrol_goals_;
    //! 首选寻敌区数量
    uint8_t amount_of_patrol_goals;
    //! 巡逻位置
    std::vector<geometry_msgs::PoseStamped> search_region_1_;
    uint8_t amount_of_search_region_1;
    std::vector<geometry_msgs::PoseStamped> search_region_2_;
    uint8_t amount_of_search_region_2;
    std::vector<geometry_msgs::PoseStamped> search_region_3_;
    uint8_t amount_of_search_region_3;
    //! 加成区监视位置 //* 敌方尚未激活加成区时候，监视位置[6个位置F1-6均有可能是加成区]
    geometry_msgs::PoseStamped buff_monitor_pose[6];

//done-------------- 己方机器人信息  --------------
    //! 机器人ID
    uint8_t ID;
    //! 自身颜色种类
    int self_color = -1, self_id = -1; //self_color: 0-red 1-blue self_id: 0-car1 1-car2
    uint16_t robot_bullets[2][2]; //GameRobotBullet提供
    uint16_t robot_hp[2][2]; //GameRobotHP提供
    //! 自身颜色以及编号
    ROBORT_COLOR_NUMBER my_color_number;
    //! 自身血量
    uint16_t my_robot_HP;
    //! 裁判系统给出的四个机器人的血量信息
    uint16_t referee_sysytem_my_hp;
    //! 自身位姿
    geometry_msgs::PoseStamped my_Pose;
    //! 自身弹丸数量
    uint16_t my_bullets;
    //! 自身是否正在被攻击
    bool if_being_attacked;
    //! 损伤扣血部位、装甲板受伤及编号
    roborts_msgs::RobotDamage robot_damage;
    //! 血量扣除信息是否更新
    bool if_robot_damage_update;
    //! 本车启动区的位姿
    geometry_msgs::PoseStamped my_boot_pose;
    //! 巡逻开始时间
    BT::TimePoint patrol_start_time;
    //! 云台角度
    float gimbal_angle_yaw;
    float gimbal_angle_pitch;
    //! 云台是否可操控
    bool gimbal_enable;
    //! 底盘是否可操控
    bool chassis_enable;
    //! 枪口是否可操控
    bool shooter_enable;
    //! 枪口信息-枪口射击频率
    uint8_t my_shoot_frequency;
    //! 枪口信息-子弹射速
    float my_shoot_speed;
    //! 枪口热量
    uint16_t shooter_heat;
    //! 热量冷却速率
    uint16_t heat_cooling_rate;
    //! 热量冷却限制
    uint16_t heat_cooling_limit; 
    //! AI机器人运动目标位置
    geometry_msgs::PoseStamped my_goal;
    //! 若处于加成区中，所处加成区的编号
    uint8_t relative_addition_area_number;
    //! 里程计信息
    nav_msgs::Odometry odom_info_;

    float gimbal_yaw_offset;
    float gimbal_pitch_offset;

    // done-------------- 友军信息  --------------
    //! 友军血量
    uint16_t friend_HP;
    //! 友军子弹数量
    uint16_t friend_Bullets;
    //! 友军位姿
    geometry_msgs::PoseStamped friend_Pose;
    bool if_know_friend_pose;
    //! 友军故障状态
    FaultType friend_FaultType; //
    //! 友军阵亡与否
    bool friend_dead;

//done-------------- 加成/惩罚区相关 数据 & 函数  --------------
    //! 加成区是否存在
    bool if_addition_exist;
    //! 加成区是否更新
    bool if_addition_update;
    //! 加成区域及状态
    roborts_msgs::GameZoneArray game_zone_array;
    //! 欲前往的目标加成区索引 //[F1,F2,F3,F4,F5,F6]
    uint8_t my_addition_goal_index; //* 从0编号

    //! 血量加成区域编号 //[F1,F2,F3,F4,F5,F6]
    uint8_t HP_Recovery;    //* 血量加成区域编号 从1编号
    //! 血量加成区激活状态
    bool if_HP_Recovery_Active;
    //! 血量加成区的位姿
    geometry_msgs::PoseStamped HP_Recovery_Pose;
    //! 子弹加成区域编号 //[F1,F2,F3,F4,F5,F6]
    uint8_t Bullets_Supply;
    //! 子弹加成区激活状态
    bool if_Bullets_Supply_Active;
    //! 子弹加成区的位姿
    geometry_msgs::PoseStamped Bullets_Supply_Pose;
    //! 敌方血量加成区域编号 //[F1,F2,F3,F4,F5,F6]
    uint8_t Enemy_HP_Recovery;
    //! 敌方血量加成区激活状态
    bool if_Enemy_HP_Recovery_Active;
    //! 敌方子弹加成区域编号 //[F1,F2,F3,F4,F5,F6]
    uint8_t Enemy_Bullets_Supply;
    //! 敌方子弹加成区激活状态
    bool if_Enemy_Bullets_Supply_Active;

//done-------------- 惩罚区  --------------
    //! 禁止移动区 //[F1,F2,F3,F4,F5,F6]
    uint8_t Disable_Moving_Area;
    //! 禁止射击区 //[F1,F2,F3,F4,F5,F6]
    uint8_t Disable_Shooting_Area;

//done-------------- 攻击敌人相关 数据 & 函数  --------------

    //* 移动目标
    geometry_msgs::PoseStamped enemy_area_goal;

    //! 敌方机器人
    EnemyInfo* enemies[2];
    //! 是否检测到敌人
    bool if_enemy_detected;
    //! 是否检测到敌方装甲板
    bool if_enemy_armor_detected;
    uint16_t enemy_1_hp = 2000;
    uint16_t enemy_2_hp = 2000;

    //! 来自哨岗的四个车的位姿消息
    roborts_msgs::CarMsgs guard_cars_msg_;

    EnemyInfo* enemy_suitable_shoot;
    double enemy_distance = 1000.0;
    int enemy_armor_area = 0;
    geometry_msgs::Point choose_armor_in_camera_;
    // 射击次数
    uint8_t shoot_number;
    uint8_t shoot_seq;
    ros::Time recent_armor_detected_time;
    roborts_msgs::ShootControl shoot_control_msg_;

    //* 云台是否调整记录器
    std::deque<uint8_t> gimbal_adjust_history_;
    //* 云台调整记录器的记录范围（帧）
    uint8_t gimbal_adjust_history_range_ = 1;
    // done ---------------
    //* 检测敌人装甲板记录器
    std::deque<uint8_t> armor_detected_history_;
    //* 装甲板记录器记录范围
    uint8_t armor_detected_history_range_ = 1;
    //* 装甲板记录检测--阈值
    uint8_t armor_detected_history_threshold_ = 1;
    //done------------
    //* 敌人检测记录器
    std::deque<uint8_t> enemy_detected_history_;
    //* 敌人检测记录器记录范围
    uint8_t enemy_detected_histoy_range_ = 1;
    //* 敌人检测记录器--阈值
    uint8_t enemy_detected_histoy_threshold_ = 1;
    //done -----------
    //* 敌人front姿势检测历史记录器
    std::deque<uint8_t> enemy_front_pose_history_;
    //* 敌人front姿势检测历史记录器范围
    uint8_t enemy_front_pose_history_range_ = 1;
    //* 敌人front姿势检测历史记录器--阈值
    uint8_t enemy_front_pose_history_threshold_ = 1;
    //* 射击的限制距离
    double shoot_distance_limit_ = 3;
    //* 正在绕后中
    bool search_back_status = false;

    // done--------------  逃跑  --------------
    

    /**
     * @brief transform the pose in camera_link frame to map frame
     * 
     * @param armor_pose_ the pose in camera frame
     * @param global_pose the pose in map frame
     * @return true if transform successfully
     * @return false 
     */
    bool GetArmorPose(geometry_msgs::Point &armor_pose_, geometry_msgs::PoseStamped &global_pose);

    //done 开启循环
    void StartSpin();

private:
//done-------------- 配置文件  --------------
    //? 配置文件路径
    std::string parma_file_path = "../parameters/decision_goal.prototxt";

//done-------------- 裁判系统  --------------
    //? 获取裁判系统消息
    //! nodeHandle
    ros::NodeHandle nh;
    //? 射击控制 topic publisher
    ros::Publisher shoot_control_publisher_;
    //? 广播信息
    ros::Subscriber ros_sub_referee_HP_;        //! 血量 订阅
    ros::Subscriber ros_sub_referee_Bullets_;   //! 子弹数量 订阅
    ros::Subscriber ros_sub_referee_GameZoneArray_;     //! 加成/惩罚区域 订阅
    ros::Subscriber ros_sub_referee_GameStatus_;        //! 比赛阶段 订阅
    ros::Subscriber ros_sub_referee_LurkStatus_;        //! 潜伏状态 订阅
    //? 自身信息
    ros::Subscriber ros_sub_referee_Shoot_info_;    //! 机器人自身枪口信息 订阅
    ros::Subscriber ros_sub_referee_Robot_status_;  //! 机器人状态信息：血量、冷却、地盘/云台/射击enable状态 订阅
    ros::Subscriber ros_sub_referee_Robot_damage_;  //! 机器人损伤扣血情况，被击打的装甲及编号 订阅
    ros::Subscriber ros_sub_referee_Robot_heat_;    //! power and heat data 热量信息 订阅

    //? 自身定位、识别检测信息   --- # 非裁判系统
    ros::Subscriber ros_sub_robot_pose_;    //! 定位信息
    ros::Subscriber ros_sub_robot_detect_;  //! 识别敌方信息
    ros::Subscriber ros_sub_robot_gimbal_angle_;    //! 未初始化
    ros::Subscriber ros_sub_robot_friendpose_;  //! 友军
    ros::Subscriber ros_sub_guard_cars_;    //! 哨岗消息--四个车的位置
    ros::Subscriber ros_sub_robot_odom_;    //! 自身里程计odom信息

    //? 底盘执行器和云台执行器
    ChassisExecutor* chassis_executor_;
    GimbalExecutor *gimbal_executor_;
    //? 调整云台
    GimbalContrl gimbal_control_;
    roborts_msgs::GimbalAngle gimbal_angle_msgs_;

    roborts_msgs::GimbalAngle gimbal_cmd_info;

    //done ******test shoot******

    bool have_last_armor_;
    
    int enermy_color_ = EnemyColor::RED;

    //if the area of the currently use armor is smaller than this value, then re-find the biggest armor, else use last armor for shoot;
    float min_area_for_resample = 800;
    float min_dis_for_resample = 0.4;

    //*抛物线高度
    float h = 0.2;
    float init_k_ = 0.026;
    float GRAVITY = 9.78;

    //* x秒检测不到目标则判定为目标丢失
    uint8_t detection_lost_limit = 0;
    //* 判断选择了新的装甲板的阈值
    uint8_t armor_change_threshold = 0.1;
    //* 记录同一装甲板被选择的次数 >= 2
    uint8_t armor_choosed_times = 0;
    //* 渐进调整云台角度
    float pinch_param = 0.5;
    //* 云台调整偏移量
    float offset_x = 0.0;   //* 弹道模型相机视角x坐标偏移量
    float offset_y = 0.0;   //* 弹道模型相机视角y坐标偏移量
    float offset_z = 0.0;   //* 弹道模型相机视角z坐标偏移量
    float offset_yaw = 0.0; //* 弹道模型 计算得到的yaw轴 的矫正偏移量
    float offset_pitch = 0.0;   //* 弹道模型 计算得到的pitch轴 的矫正偏移量
    float offset_y_weight = 0.0;    //* detection y pose offset weight
    //* 云台h调整参数
    float h_adjust = 0.0;

    //* 卡尔曼滤波
    //KF_Predict::KF_Predictor kf_predictor(4,2);
    KF_Predict::KF_Predictor *kf_predictor_p_ = new KF_Predict::KF_Predictor(4, 2);

    // minimal distance between bullet path and obstacle (in cells)
    int bullet_min_cell_dis = 1; // equals cell_dis * 0.05m

    //memory last yaw last pitch;
    float last_yaw_ = 0.0;
    float last_pitch_ = 0.0;

    //the threshold for changing yaw and pitch
    float yaw_change_threshold = 0.02;
    float pitch_change_threshold = 0.02;

    //the minimal step to control yaw;
    float yaw_change_unit_ = 0.05;

    //
    bool enable_shoot;

    // done ----------  函数定义  ----------
    /**
      * ? 相关函数定义
      * ! blackboard中只实现 blackboard数据 必要依赖的相关函数
    / */
    /**
     //done ros::Sub  CallBacks ---- 裁判系统消息回调函数  ----------
     */
    //* 血量订阅的回调函数
    void Sub_GameRobotHP_CallBack_(const roborts_msgs::GameRobotHP::ConstPtr& game_robot_hp);
    //* 子弹数量订阅的回调函数
    void Sub_GameRobotBullet_CallBack_(const roborts_msgs::GameRobotBullet::ConstPtr& game_robot_bullet);
    //* 加成/惩罚区域数组 订阅回调函数
    void Sub_GameZoneArrayStatus_CallBack_(const roborts_msgs::GameZoneArray::ConstPtr& game_zone_array_status);
    //* 比赛阶段订阅回调函数
    void Sub_GameStatus_CallBack_(const roborts_msgs::GameStatus::ConstPtr& game_status);
    //* 潜伏机制订阅回调函数
    void Sub_GameLurkStatus_CallBack_(const roborts_msgs::LurkStatus::ConstPtr& lurk_status);
    //* 机器人自身状态订阅回调函数
    void Sub_RobotStatus_CallBack_(const roborts_msgs::RobotStatus::ConstPtr& robot_status);
    //* 机器人损伤扣血情况：装甲板被击打及编号、超限等订阅回调函数
    void Sub_RobotDamage_CallBack_(const roborts_msgs::RobotDamage::ConstPtr& robot_damage);
    //* 机器人热量信息订阅回调函数
    void Sub_RobotHeat_CallBack_(const roborts_msgs::RobotHeat::ConstPtr& robot_heat);
    //* 机器人枪口信息订阅回调函数
    void Sub_RobotShootInfo_CallBack_(const roborts_msgs::RobotShoot::ConstPtr& robot_shoot);

    //done ----------  AI机器人自身相关方法实现  ----------
    //* 回调函数--机器人自身位置信息的回调函数
    void Sub_RobotPose_CallBack_(const geometry_msgs::PoseStamped::ConstPtr& robot_pose);
    //* 回调函数--机器人识别到敌人信息的回调函数
    void Sub_RobotDetectEnemy_CallBack_(const roborts_msgs::ArmorMsgs::ConstPtr& robot_detect);
    //* 回调函数--机器人友军威姿订阅回调函数
    void Sub_Robot_FriendPose_CallBack_(const geometry_msgs::PoseStamped::ConstPtr& robot_pose);
    //* 回调函数--机器人订阅哨岗消息
    void Sub_Guard_Cars_CallBack_(const roborts_msgs::CarMsgs::ConstPtr &car_msgs);
    //* 回调函数--机器人自身的odom里程计消息订阅回调函数
    void Sub_Robot_Odom_CallBack_(const nav_msgs::Odometry::ConstPtr &odom_info);

//todo ******************* test ********************

    /**
     * @brief Calculate the actual y value with air resistance
     * @param x the distanc
     * @param v Projectile velocity
     * @param angle Pitch angle
     * @return The actual y value in the gimbal coordinate
     */
    float BulletModel(float x, float v, float angle);
    /**
     * @brief Get the gimbal control angle
     * @param x Distance from enemy(the armor selected to shoot) to gimbal
     * @param y Value of y in gimbal coordinate.
     * @param v Projectile velocity
     * @return Gimbal pitch angle
     */
    float GetPitch(float x, float y, float v);
    /**
     * @brief check whether the line (start-end) cross the "high" obstacles.
     * 
     * @param startx the x value of start poin
     * @param starty the y value of start point
     * @param endx  the x value of end point
     * @param endy the y value of end point
     * @return true if the line cross high obstacles
     * @return false else
     */
    bool CheckCrossWithHighObstacle(geometry_msgs::PoseStamped start,geometry_msgs::PoseStamped end);
    /**
     * @brief calculate the y value of the line (pass point(x, y) and slope equals k) where x value equals (int)(x + 1)
     * 
     * @param x
     * @param y 
     * @param k 
     * @return double y value in (int)(x + 1)
     */
    double GetNextY(double x, double y, double k);

    /**
     * @brief check whether a vertical line cross any high obstacle
     * 
     * @param x 
     * @param y_1 
     * @param y_2 
     * @return true don't cross
     * @return false else
     */
    bool CheckVerticalLineCross(int x, int y_1, int y_2);


    /**
     * @brief choose the optimal armor for shoot and the optimal number of bullets to shoot
     * 
     * @param msgs armor msgs
     * @param choosed_armor the choosed optimal armor for shoot
     * @param shoot_num the number of bullets for shoot
     * @return true if can shoot
     * @return false else
     */
    bool ChooseArmorForShoot(std::vector<roborts_msgs::ArmorMsg> msgs, roborts_msgs::ArmorMsg &choosed_armor, int &shoot_num);

    /**
     * @brief calculate the distance between two points.
     * 
     * @param p_1 the first point
     * @param p_2 the second point
     * @return double 
     */
    double Distance(geometry_msgs::PoseStamped &p_1, geometry_msgs::PoseStamped &p_2);

    
};





#endif