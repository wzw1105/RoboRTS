#include "./blackboard.h"
#include "io/io.h"
#include <ros/ros.h>
#include <math.h>
#include "tool/tool.h"

//玻璃信息
int grass_obstacle_info[4][4] = {
    {142, 20, 21, 5},
    {128, 70, 5, 20},
    {30, 1, 5, 21},
    {1, 66, 20, 5}};

EnemyInfo::EnemyInfo(uint16_t bullets_){
    this->bullets = bullets_;
    this->if_detected = false;
    this->distance = 0;
    this->HP = 2000;
    this->hit_me_times = 0;
    this->if_main_attacker = false;
    this->main_attacker_coefficient = 0;
    this->if_has_pose_info = false;

}

EnemyInfo::~EnemyInfo() {}

//* BlackBoard初始化
BlackBoard::BlackBoard(ChassisExecutor*& chassis_executor, GimbalExecutor*& gimbal_executor) {

    this->game_end_ = false;

    //* tf
    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
    //* 代价地图
    std::string map_path = ros::package::getPath("roborts_costmap") +
                           "/config/costmap_parameter_config_for_decision.prototxt";
    //* 代价地图的共享指针
    costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_,
                                             map_path);
    //* char map
    charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();

    costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();

    //* 配置文件路径
    this->param_file_path = ros::package::getPath("decision") + "/src/config/decision.prototxt";

    // done-------------- 从配置文件加载参数  ---------------
    RTS_DECISION::DecisionConfig decision_params;
    if (!roborts_common::ReadProtoFromTextFile(this->param_file_path, &decision_params))
    {
        std::cout << "读取解析  配置文件出错了" << std::endl;
    }
    else {
        //*  启动区初始化
        my_boot_pose.pose.position.x = decision_params.master_bot().start_position().x();
        my_boot_pose.pose.position.y = decision_params.master_bot().start_position().y();
        my_boot_pose.pose.position.z = decision_params.master_bot().start_position().z();
        my_boot_pose.pose.orientation =
            tf::createQuaternionMsgFromYaw(decision_params.master_bot().start_position().yaw());
        enemy_area_goal = my_boot_pose;

        this->friend_Pose.pose.position.x = my_boot_pose.pose.position.x;
        this->friend_Pose.pose.position.y = my_boot_pose.pose.position.y;
        this->friend_Pose.pose.position.z = my_boot_pose.pose.position.z;
        this->friend_Pose.header.stamp = ros::Time::now();
        this->friend_Pose.pose.orientation = my_boot_pose.pose.orientation;

        //* 巡逻区域初始化
        this->patrol_goals_.resize(decision_params.patrol_region().size());
        this->amount_of_patrol_goals = decision_params.patrol_region().size();
        for (int i = 0; i < decision_params.patrol_region().size(); i++)
        {
            this->patrol_goals_[i].header.frame_id = "map";
            this->patrol_goals_[i].pose.position.x = decision_params.patrol_region(i).x();

            this->patrol_goals_[i].pose.position.y = decision_params.patrol_region(i).y();
            this->patrol_goals_[i].pose.position.z = decision_params.patrol_region(i).z();
            this->patrol_goals_[i].pose.orientation =
                tf::createQuaternionMsgFromYaw(decision_params.patrol_region(i).yaw());
        }

        //* 3个巡逻区域配置
        this->search_region_1_.resize(decision_params.search_region_1().size());
        this->amount_of_search_region_1 = decision_params.search_region_1().size();
        for (int i = 0; i < decision_params.search_region_1().size(); i++)
        {
            this->search_region_1_[i].header.frame_id = "map";
            this->search_region_1_[i].pose.position.x = decision_params.search_region_1(i).x();
            this->search_region_1_[i].pose.position.y = decision_params.search_region_1(i).y();
            this->search_region_1_[i].pose.position.z = decision_params.search_region_1(i).z();
            this->search_region_1_[i].pose.orientation =
                tf::createQuaternionMsgFromYaw(decision_params.search_region_1(i).yaw());
        }
        //* 3个巡逻区域配置
        this->search_region_2_.resize(decision_params.search_region_2().size());
        this->amount_of_search_region_2 = decision_params.search_region_2().size();
        for (int i = 0; i < decision_params.search_region_2().size(); i++)
        {
            this->search_region_2_[i].header.frame_id = "map";
            this->search_region_2_[i].pose.position.x = decision_params.search_region_2(i).x();
            this->search_region_2_[i].pose.position.y = decision_params.search_region_2(i).y();
            this->search_region_2_[i].pose.position.z = decision_params.search_region_2(i).z();
            this->search_region_2_[i].pose.orientation =
                tf::createQuaternionMsgFromYaw(decision_params.search_region_2(i).yaw());
        }
        //* 3个巡逻区域配置
        this->search_region_3_.resize(decision_params.search_region_3().size());
        this->amount_of_search_region_3 = decision_params.search_region_3().size();
        for (int i = 0; i < decision_params.search_region_3().size(); i++)
        {
            this->search_region_3_[i].header.frame_id = "map";
            this->search_region_3_[i].pose.position.x = decision_params.search_region_3(i).x();
            this->search_region_3_[i].pose.position.y = decision_params.search_region_3(i).y();
            this->search_region_3_[i].pose.position.z = decision_params.search_region_3(i).z();
            this->search_region_3_[i].pose.orientation =
                tf::createQuaternionMsgFromYaw(decision_params.search_region_3(i).yaw());
        }

        //* 6个加成区初始化 //? F1 ~ F6 区域
        for (int i = 0; i < 6; i++)
        {
            this->F_1_6[i].header.frame_id = "map";
            this->F_1_6[i].pose.position.x = decision_params.buff_point(i).x();
            this->F_1_6[i].pose.position.y = decision_params.buff_point(i).y();
            this->F_1_6[i].pose.position.z = decision_params.buff_point(i).z();
            this->F_1_6[i].pose.orientation =
                tf::createQuaternionMsgFromYaw(decision_params.buff_point(i).yaw());
        }

        //* 子弹数量初始化
        // this->my_bullets = decision_params.master() ? 50 : 0;
        //* 检测 判定为敌人丢失的时间阈值
        this->detection_lost_limit = decision_params.detection_shoot_config().detect_lost_limit();
        //* 判断选择装甲板改变的|x+y| --阈值
        this->armor_change_threshold = decision_params.detection_shoot_config().armor_change_threshold();
        //* 云台调整历史记录器的记录范围
        this->gimbal_adjust_history_range_ = decision_params.detection_shoot_config().gimbal_adjust_history_range();
        //* 装甲板检测历史记录器的-记录范围
        this->armor_detected_history_range_ = decision_params.detection_shoot_config().armor_detected_history_range();
        //* 装甲板检测历史记录器--阈值
        this->armor_detected_history_threshold_ = decision_params.detection_shoot_config().armor_detected_history_threshold();
        //* 敌人检测历史记录器的-记录范围
        this->enemy_detected_histoy_range_ = decision_params.detection_shoot_config().enemy_detected_history_range();
        //* 敌人检测历史记录器--阈值
        this->enemy_detected_histoy_threshold_ = decision_params.detection_shoot_config().enemy_detected_history_threshold();
        //* 敌人front姿势历史记录器的-记录范围
        this->enemy_front_pose_history_range_ = decision_params.detection_shoot_config().enemy_front_pose_history_range();
        //* 敌人front姿势历史记录器--阈值
        this->enemy_front_pose_history_threshold_ = decision_params.detection_shoot_config().enemy_front_pose_history_threshold();
        //* 射击限制距离
        this->shoot_distance_limit_ = decision_params.detection_shoot_config().shoot_distance_limit();
        //* 云台调整阈值
        this->yaw_change_threshold = decision_params.camera_gimbal_transform().yaw_change_threshold();
        this->pitch_change_threshold = decision_params.camera_gimbal_transform().pitch_change_threshold();

        //* 逼近云台角度调整的速率
        this->pinch_param = decision_params.camera_gimbal_transform().pinch_param();
        //* 云台调整---偏移量
        this->offset_x = decision_params.camera_gimbal_transform().offset_x();
        this->offset_y = decision_params.camera_gimbal_transform().offset_y();
        this->offset_z = decision_params.camera_gimbal_transform().offset_z();
        this->offset_yaw = decision_params.camera_gimbal_transform().offset_yaw();
        this->offset_pitch = decision_params.camera_gimbal_transform().offset_pitch();
        this->offset_y_weight = decision_params.camera_gimbal_transform().offset_y_weight();

        //* 弹道模型参数
        this->GRAVITY = decision_params.shooter_model_config().gravity();
        this->h = decision_params.shooter_model_config().h();
        this->init_k_ = decision_params.shooter_model_config().init_k_();
        this->h_adjust = decision_params.shooter_model_config().h_adjust();
        this->my_shoot_speed = decision_params.shooter_model_config().shoot_speed();

        //* PID控制器
        this->pid_kp = decision_params.camera_gimbal_transform().pid_kp();
        this->pid_ki = decision_params.camera_gimbal_transform().pid_ki();
        this->pid_kd = decision_params.camera_gimbal_transform().pid_kd();
        //* 是否启用
        this->use_pid = decision_params.use_pid();
    }

    // done-------------- 裁判系统相关 订阅节点初始化  --------------
    //? 初始化订阅节点
    //* 比赛信息 Game Related 血量、子弹量、加成/惩罚区域、比赛阶段、潜伏期
    this->ros_sub_referee_HP_ = this->nh.subscribe("game_robot_hp", 3, &BlackBoard::Sub_GameRobotHP_CallBack_, this);                             // advertise<roborts_msgs::GameRobotHP>
    this->ros_sub_referee_Bullets_ = this->nh.subscribe("game_robot_bullet", 3, &BlackBoard::Sub_GameRobotBullet_CallBack_, this);                // advertise<roborts_msgs::GameRobotBullet>("game_robot_bullet",3);
    this->ros_sub_referee_GameZoneArray_ = this->nh.subscribe("game_zone_array_status", 3, &BlackBoard::Sub_GameZoneArrayStatus_CallBack_, this); // advertise<roborts_msgs::GameZoneArray>("game_zone_array_status",3);
    this->ros_sub_referee_GameStatus_ = this->nh.subscribe("game_status", 3, &BlackBoard::Sub_GameStatus_CallBack_, this);                        // advertise<roborts_msgs::GameStatus>("game_status",3);
    this->ros_sub_referee_LurkStatus_ = this->nh.subscribe("game_lurk_status", 3, &BlackBoard::Sub_GameLurkStatus_CallBack_, this);               // advertise<roborts_msgs::LurkStatus>("game_lurk_status",3);
    //* 机器人自身信息 Robot Related 机器人状态、扣血、热量、枪口信息
    this->ros_sub_referee_Robot_status_ = this->nh.subscribe("robot_status", 3, &BlackBoard::Sub_RobotStatus_CallBack_, this);
    this->ros_sub_referee_Robot_damage_ = this->nh.subscribe("robot_damage", 3, &BlackBoard::Sub_RobotDamage_CallBack_, this);
    this->ros_sub_referee_Robot_heat_ = this->nh.subscribe("robot_heat", 3, &BlackBoard::Sub_RobotHeat_CallBack_, this);
    this->ros_sub_referee_Shoot_info_ = this->nh.subscribe("robot_shoot", 3, &BlackBoard::Sub_RobotShootInfo_CallBack_, this);
    //* 机器人自身信息 非裁判系统  定位 & 检测识别信息
    this->ros_sub_robot_pose_ = this->nh.subscribe("amcl_pose", 3, &BlackBoard::Sub_RobotPose_CallBack_, this);
    this->ros_sub_robot_detect_ = this->nh.subscribe("armors_info", 3, &BlackBoard::Sub_RobotDetectEnemy_CallBack_, this);
    // //* 云台角度
    //this->ros_sub_robot_gimbal_angle_ = this->nh.subscribe("cmd_gimbal_angle", 3, &BlackBoard::Sub_GimbalAngle_CallBack_, this);
    //* 友军位姿
    this->ros_sub_robot_friendpose_ = this->nh.subscribe("friend_pose", 3, &BlackBoard::Sub_Robot_FriendPose_CallBack_, this);
    //* 哨岗消息- 四个车的位姿
    this->ros_sub_guard_cars_ = this->nh.subscribe("pose_from_post", 3, &BlackBoard::Sub_Guard_Cars_CallBack_, this);
    //* 里程计信息
    this->ros_sub_robot_odom_ = this->nh.subscribe("odom", 1, &BlackBoard::Sub_Robot_Odom_CallBack_, this);

    //* 发布云台调整消息
    this->shoot_control_publisher_ = this->nh.advertise<roborts_msgs::ShootControl>("shoot_control", 1);

    //* 底盘执行器和云台执行器
    this->chassis_executor_ = chassis_executor;
    this->gimbal_executor_ = gimbal_executor;
    //* 云台角度消息msg
    this->gimbal_angle_msgs_.yaw_mode = true;
    this->gimbal_angle_msgs_.pitch_mode = false;

    // done-------------- 比赛相关  --------------
    //* 潜伏期
    this->lurk_mode = LurkingStatus::Normal;
    this->game_status = GameStatus::Ready;
    this->game_remaining_time = 180; // seconds

    // done-------------- 初始化己方相关信息数据等  --------------
    //* guardmsg
    this->guard_cars_msg_.stamp_guard = ros::Time::now();
    //* 自身ID
    this->ID = 1;
    //* 自身血量
    this->my_robot_HP = 2000;
    this->referee_sysytem_my_hp = 2000;
    //* 自身位姿
    //# this->my_Pose; 不进行初始化
    //* 自身是否正在被攻击
    this->if_being_attacked = false;
    //* 损伤血量扣除
    //# this->robot_damage 不进行初始化
    //* 巡逻开始时间
    BT::TimePoint patrol_start_time;
    //* 云台角度
    this->gimbal_angle_pitch = 0; // pitch轴
    this->gimbal_angle_yaw = 0;   // yaw轴
    //* 初始化自身子弹数量
    this->my_bullets = 0;

    //* 初始化枪口、底盘、云台是否可操控
    this->gimbal_enable = true;
    this->chassis_enable = true;
    this->shooter_enable = true;
    //* 枪口射频、射速、热量
    this->my_shoot_frequency = 0;
    this->my_shoot_speed = 0;
    this->shooter_heat = 0;
    //* 枪口冷却速率、限制
    this->heat_cooling_rate = 0;
    this->heat_cooling_limit = 0;
    //* 若处于加成区中，所处加成区的编号
    this->relative_addition_area_number = 0;

    this->gimbal_yaw_offset = 0.0;
    this->gimbal_pitch_offset = 0.0;

    // done-------------- 初始化友军信息 数据等  --------------
    //* 友军故障情况
    this->friend_FaultType = FaultType::None;
    //* 友军血量
    this->friend_HP = 2000;
    //* 友军子弹数量
    this->friend_Bullets = 0;
    //* 友军位姿
    //# this->friend_Pose 不进行初始化
    this->if_know_friend_pose = false;
    //* 友军阵亡与否
    this->friend_dead = false;

    // done-------------- 初始化加成区相关数据信息等  --------------
    //* 加成区是否存在
    this->if_addition_exist = false;
    //* 加成区是否更新
    this->if_addition_update = false;
    //* 加成区域及状态
    //# ths->game_zone_array; 不进行初始化
    //* 目标加成区编号
    this->my_addition_goal_index = 6;

    //* 血量加成区编号 //[F1,F2,F3,F4,F5,F6]
    this->HP_Recovery = 0;
    //* 血量加成区激活状态
    this->if_HP_Recovery_Active = false;
    //* 子弹加成区编号 //[F1,F2,F3,F4,F5,F6]
    this->Bullets_Supply = 0;
    //* 子弹加成区激活状态
    this->if_Bullets_Supply_Active = false;
    //* 敌方血量加成区编号
    this->Enemy_HP_Recovery = 0;
    //* 敌方血量加成区激活状态
    this->if_Enemy_HP_Recovery_Active = false;
    //* 敌方子弹加成区编号
    this->Enemy_Bullets_Supply = 0;
    //* 敌方子弹加成激活状态
    this->if_Enemy_Bullets_Supply_Active = false;
    //* 禁止移动区域编号
    this->Disable_Moving_Area = 0;
    //* 禁止射击区域编号
    this->Disable_Shooting_Area = 0;

    // done-------------- 初始化敌人数据信息等  --------------
    //*创建敌人 Create Enemy
    this->enemies[0] = new EnemyInfo(50);
    this->enemies[1] = new EnemyInfo(0);
    //* 是否检测到敌人
    this->if_enemy_detected = false;
    //* 是否检测到敌人装甲板
    this->if_enemy_armor_detected = false;

    // done 初始化一些条件************测试
    this->enemy_distance = 1000.0;
    this->shoot_seq = 0;
    this->recent_armor_detected_time = ros::Time::now();
    this->shoot_control_msg_.yaw_angle = 0;
    this->shoot_control_msg_.pitch_angle = 0;
    //* 云台记录器
    this->gimbal_adjust_history_.assign(this->gimbal_adjust_history_range_, 0);
    //* 装甲板检测记录器
    this->armor_detected_history_.assign(this->armor_detected_history_range_, 0);
    //* 敌人检测记录器
    this->enemy_detected_history_.assign(this->enemy_detected_histoy_range_,0);
    //* 敌人front姿势检测历史记录器
    this->enemy_front_pose_history_.assign(this->enemy_front_pose_history_range_, 0);
    //* 卡尔曼预测器
    this->kf_predictor_p_ = new KF_Predict::KF_Predictor(4, 2);
}

BlackBoard::~BlackBoard(){
    std::cout << "~BlackBoard()" << std::endl;
}

/**
 * @brief 代价地图
 * ? CostMap引用
 */
const std::shared_ptr<CostMap> BlackBoard::GetCostMap() {
    return costmap_ptr_;
}

/**
 * @brief costmap 2D
 */
const CostMap2D* BlackBoard::GetCostMap2D() {
    return costmap_2d_;
}
/**
 * @brief CharMap
 */
const unsigned char* BlackBoard::GetCharMap() {
    return charmap_;
}

/**
 * @brief Get the Gimbal Yaw Angle object
 *
 * @return float
 */
float BlackBoard::getGimbalYawAngle(){
    tf::StampedTransform listenTransform;
    this->tf_ptr_->lookupTransform("base_link", "gimbal", ros::Time(0), listenTransform);
    this->gimbal_angle_yaw = tf::getYaw(listenTransform.getRotation());
}

/**
 * @brief BlackBoard类中方法实现
 * ! 函数定义
 */
SWING_STATUS BlackBoard::ComputeSwingStatus() {
    return SWING_STATUS::SLOW;
}

//? 获取本机器人的启动区位姿
geometry_msgs::PoseStamped BlackBoard::getMyBootPose(){
    //* 补充代码
    //* end
    return this->my_boot_pose;
}


/**
* @brief 裁判系统 Referee System
//done ----------------- 裁判系统  -----------------
*/
//? 血量订阅的回调函数
void BlackBoard::Sub_GameRobotHP_CallBack_(const roborts_msgs::GameRobotHP::ConstPtr &game_robot_hp){
    //* 裁判系统 比赛中的血量信息
    //* 消息收发频率 1Hz
    this -> robot_hp[0][0] = game_robot_hp -> red1;
    this -> robot_hp[0][1] = game_robot_hp -> red2;
    this -> robot_hp[1][0] = game_robot_hp -> blue1;
    this -> robot_hp[1][1] = game_robot_hp -> blue2;

    /**
     * @brief 危险写法
     */
    uint16_t last_referee_sysytem_my_hp = this->referee_sysytem_my_hp;
    switch (this->ID)
    {
    case 1: //* 红方1号
        this->referee_sysytem_my_hp = game_robot_hp->red1;
        this->friend_HP = game_robot_hp->red2;
        if (this->enemy_1_hp < game_robot_hp->blue1)
        {
            this->my_Hittig = this->my_Hittig +  game_robot_hp->blue1 - this->enemy_1_hp;
        }
        if (this->enemy_2_hp < game_robot_hp->blue2)
        {
            this->my_Hittig = this->my_Hittig + game_robot_hp->blue2 - this->enemy_2_hp;
        }
        this->enemy_1_hp = game_robot_hp->blue1;
        this->enemy_2_hp = game_robot_hp->blue2;
        break;
    case 2: //* 红方2号
        this->referee_sysytem_my_hp = game_robot_hp->red2;
        this->friend_HP = game_robot_hp->red1;
        break;
    case 101: //* 蓝方1号
        this->referee_sysytem_my_hp = game_robot_hp->blue1;
        this->friend_HP = game_robot_hp->blue2;
        break;
    case 102: //* 蓝方2号
        this->referee_sysytem_my_hp = game_robot_hp->blue2;
        this->friend_HP = game_robot_hp->blue1;
        break;
      default:
          break;
      }

    //* 判断友军死亡否
    if (this->friend_HP == 0)
        this->friend_dead = true;
    
    //* 判断是否被击打
    

}

//? 子弹数量订阅的回调函数
void BlackBoard::Sub_GameRobotBullet_CallBack_(const roborts_msgs::GameRobotBullet::ConstPtr &game_robot_bullet){
    //* 话题消息收发频率 1 Hz
    this -> robot_bullets[0][0] = game_robot_bullet -> red1;
    this -> robot_bullets[0][1] = game_robot_bullet -> red2;
    this -> robot_bullets[1][0] = game_robot_bullet -> blue1;
    this -> robot_bullets[1][1] = game_robot_bullet -> blue2;
    /**
     * @brief 危险写法
     */
    switch (this->ID) {
    case 1: //* 红方1号

        this->my_bullets = game_robot_bullet->red1;

        this->friend_Bullets = game_robot_bullet->red2;
        this->enemies[0]->bullets = game_robot_bullet->blue1;
        this->enemies[1]->bullets = game_robot_bullet->blue2;
        break;
    case 2: //* 红方2号
        this->my_bullets = game_robot_bullet->red2;

        this->friend_Bullets = game_robot_bullet->red1;
        this->enemies[0]->bullets = game_robot_bullet->blue1;
        this->enemies[1]->bullets = game_robot_bullet->blue2;
        break;
    case 101: //* 蓝方1号
        this->my_bullets = game_robot_bullet->blue1;

        this->friend_Bullets = game_robot_bullet->blue2;
        this->enemies[0]->bullets = game_robot_bullet->red1;
        this->enemies[1]->bullets = game_robot_bullet->red2;
        break;
    case 102: //* 蓝方2号
        this->my_bullets = game_robot_bullet->blue2;

        this->friend_Bullets = game_robot_bullet->blue1;
        this->enemies[0]->bullets = game_robot_bullet->red1;
        this->enemies[1]->bullets = game_robot_bullet->red2;
        break;
      default:
          break;
    }
}

/**
    RED_HP_RECOVERY=1
    RED_BULLET_SUPPLY=2
    BLUE_HP_RECOVERY=3
    BLUE_BULLET_SUPPLY=4
    DISABLE_SHOOTING=5
    DISABLE_MOVEMENT=6
*/
//? 加成/惩罚区域 数组 订阅回调函数
void BlackBoard::Sub_GameZoneArrayStatus_CallBack_(const roborts_msgs::GameZoneArray::ConstPtr& game_zone_array_status) {
    //done 这里的加成区激活状态是指比赛时，该加成区是否还具有加成作用，
    //done 策略中定义的激活状态是指加成区是否被经过
    //* 话题消息收发频率 1 Hz
    uint8_t count_active_area = 0;  //* 加成区激活个数，判断加成区是否已经前往
    for (int i = 0; i < 6; i++) {
        switch (game_zone_array_status->zone[i].type)
        {
        case 1: //* RED_HP_RECOVERY=1
            if ( this->ID < 100 ) { // 我是红方
                std::cout << "红方血量加成区： " << i + 1 << std::endl;
                this->HP_Recovery = i + 1;
                if (!game_zone_array_status->zone[i].active)
                {
                    count_active_area = count_active_area + 1;
                    this->if_HP_Recovery_Active = true;
                }
                else {
                    this->if_HP_Recovery_Active = false;
                }
            }
            else { // 我是蓝方
                //* 敌方血量加成区编号 以及是否 已加成
                this->Enemy_HP_Recovery = i + 1;
                if (!game_zone_array_status->zone[i].active)
                {
                    this->if_Enemy_HP_Recovery_Active = true;
                }
                else {
                    this->if_Enemy_HP_Recovery_Active = false;
                }
            }
            break;
        case 2: //* RED_BULLET_SUPPLY=2
            std::cout << "红方子弹加成区： " << i + 1 << std::endl;
            if ( this->ID < 100 ) { // 我是红方
                //* 子弹加成区 及激活状态;
                this->Bullets_Supply = i + 1;
                if (!game_zone_array_status->zone[i].active) {
                    count_active_area = count_active_area + 1;
                    this->if_Bullets_Supply_Active = true;
                }
                else {
                    this->if_Bullets_Supply_Active = false;
                }
            }
            else { // 我是蓝方
                this->Enemy_Bullets_Supply = i + 1;
                if (!game_zone_array_status->zone[i].active) {
                    this->if_Enemy_Bullets_Supply_Active = true;
                }
                else {
                    this->if_Enemy_Bullets_Supply_Active = false;
                }                    
            }
            break;
        case 3: //* BLUE_HP_RECOVERY=3
            if ( this->ID > 100 ) { // 我是蓝方
                this->HP_Recovery = i + 1;
                if (!game_zone_array_status->zone[i].active) {
                    count_active_area = count_active_area + 1;
                    this->if_HP_Recovery_Active = true;
                }
                else {
                    this->if_HP_Recovery_Active = false;
                }
            }
            else { // 我是红方
                this->Enemy_HP_Recovery = i + 1;
                if (!game_zone_array_status->zone[i].active) {
                    this->if_Enemy_HP_Recovery_Active = true;
                }
                else {
                    this->if_Enemy_HP_Recovery_Active = false;
                }
            }
            break;
        case 4: //* BLUE_BULLET_SUPPLY=4
            if ( this->ID > 100 ) { // 我是蓝方
                this->Bullets_Supply = i + 1;
                if (!game_zone_array_status->zone[i].active) {
                    count_active_area = count_active_area + 1;
                    this->if_Bullets_Supply_Active = true;
                }
                else {
                    this->if_Bullets_Supply_Active = false;
                }
            }
            else { // 我是红方
                this->Enemy_Bullets_Supply = i+1;
                if (!game_zone_array_status->zone[i].active) {
                    this->if_Enemy_Bullets_Supply_Active = true;
                }
                else {
                    this->if_Enemy_Bullets_Supply_Active = false;
                }      
            }
            break;
        case 5: //* DISABLE_SHOOTING=5
            this->Disable_Moving_Area = i + 1;
            break;
        case 6: //* DISABLE_MOVEMENT=6
            this->Disable_Shooting_Area = i + 1;
            break;
        default:
            break;
        }
    }
    if (count_active_area < 2)
        this->if_addition_exist = true;
    else
        this->if_addition_exist = false;
    
    if (this->if_addition_exist) {
        std::cout<< "addition exist: " << 2 - count_active_area<<std::endl;
    }
}

//? 比赛阶段订阅回调函数
void BlackBoard::Sub_GameStatus_CallBack_(const roborts_msgs::GameStatus::ConstPtr& game_status) {
    //* 比赛开始、一分钟后、两分钟后，加成区随机更新
    //* 该topic的数据，发送/接收的频率为 1 Hz
    //* 比赛阶段
    this->game_remaining_time = game_status->remaining_time;
    std::cout << "remaining time: " << + game_status->remaining_time << std::endl;
    
    switch (game_status->game_status)
    {
    case 0:
        this->game_status = GameStatus::Ready;
        break;
    case 1:
        this->game_status = GameStatus::Preparation;
        break;
    case 2:
        this->game_status = GameStatus::Initialize;
        break;
    case 3:
        this->game_status = GameStatus::Five_Second_CD;
        break;
    case 4:
        this->game_status = GameStatus::Game;
        if (this->game_remaining_time == 180 ||
            this->game_remaining_time == 179 ||
            this->game_remaining_time == 120 ||
            this->game_remaining_time == 60
            ) {
            std::cout << "加成区更新: " << + this->game_remaining_time << std::endl;
            //* 加成区更新
            this->if_addition_update = true;
        }
        else {
            this->if_addition_update = false;
        }
        break;
    case 5:
        this->game_status = GameStatus::End;
        break;
    default:
        break;
    }
}

//? 潜伏机制订阅回调函数
void BlackBoard::Sub_GameLurkStatus_CallBack_(const roborts_msgs::LurkStatus::ConstPtr &lurk_status){
    // todo ********
    //* 话题消息收发频率 1 Hz
    switch (lurk_status->lurk_mode)
    {
    case 0:
        this->lurk_mode = LurkingStatus::Normal;
        break;
    case 1:
        this->lurk_mode = LurkingStatus::Ready;
        break;
    case 2:
        this->lurk_mode = LurkingStatus::Lurking;
        break;
    default:
        break;
    }
}

//? 机器人自身状态订阅回调函数
void BlackBoard::Sub_RobotStatus_CallBack_(const roborts_msgs::RobotStatus::ConstPtr& robot_status) {
    //* ID、血量、底盘/云台/枪口是否可操控
    //* 话题收发频率比较高
    uint16_t last_HP = this->my_robot_HP;
    this->my_robot_HP = robot_status->remain_hp;
    this->ID = robot_status->id;
    //* 底盘、云台、枪管是否可用
    if (!this->shooter_enable && robot_status->shooter_enable)
    {
        roborts_msgs::FricWhl fricwhl_srv_msg;
        fricwhl_srv_msg.response.received = false;
        fricwhl_srv_msg.request.open = true;
        this->gimbal_executor_->Execute(fricwhl_srv_msg);
    }

    this->self_id = (robot_status -> id % 100) - 1;
    this->self_color = robot_status -> id < 10 ? 0 : 1; //红：0 蓝：1

    this->gimbal_enable = robot_status->gimbal_enable;
    this->chassis_enable = robot_status->chassis_enable;
    this->shooter_enable = robot_status->shooter_enable;
    //* 热量冷却、限制
    this->heat_cooling_rate = robot_status->heat_cooling_rate;
    this->heat_cooling_limit = robot_status->heat_cooling_limit;

    //* 判断是否正在被击打
    if (last_HP == this->my_robot_HP) {
        if (!this->if_robot_damage_update) {
            this->if_being_attacked = false;
        }
        else {
            this->if_robot_damage_update = false;
        }
    }
    else if (last_HP > this->my_robot_HP) {
        if (this->if_robot_damage_update) {
            this->if_robot_damage_update = false;
            switch (this->robot_damage.damage_type)
            {
            case 0:
                this->if_being_attacked = true;
                break;
            case 1:
                break;
            default:
                break;
            }
        }
    }
}

//? 机器人损伤扣血情况：装甲板被击打及编号、超限等订阅回调函数
void BlackBoard::Sub_RobotDamage_CallBack_(const roborts_msgs::RobotDamage::ConstPtr& robot_damage) {
    //* 损伤部位及装甲板编号
    this->if_robot_damage_update = true;
    this->robot_damage = *robot_damage;
    if (robot_damage->damage_type == 0) {
        this->if_being_attacked = true;
    }
}

//? 机器人热量信息订阅回调函数
void BlackBoard::Sub_RobotHeat_CallBack_(const roborts_msgs::RobotHeat::ConstPtr& robot_heat) {
    //* 热量信息
    this->shooter_heat = robot_heat->shooter_heat;
}

//? 机器人枪口信息订阅回调函数
void BlackBoard::Sub_RobotShootInfo_CallBack_(const roborts_msgs::RobotShoot::ConstPtr& robot_shoot) {
    //* 枪口信息
    this->my_shoot_frequency = robot_shoot->frequency;
    this->my_shoot_speed = robot_shoot->speed;
}


//done ----------  AI机器人自身相关方法实现  ----------

//* 回调函数--机器人自身位置信息的回调函数
void BlackBoard::Sub_RobotPose_CallBack_(const geometry_msgs::PoseStamped::ConstPtr& robot_pose) {
    
    this->my_Pose = *robot_pose;
}
//* 回调函数--机器人识别到敌人信息的回调函数
void BlackBoard::Sub_RobotDetectEnemy_CallBack_(const roborts_msgs::ArmorMsgs::ConstPtr& robot_detect) {

    int shoot_num;
    roborts_msgs::ArmorMsg choosed_armor;
    double armor_distance = 10000.0;
    double distance_current = 10000.0;
    int area = 0;

    //* 云台调整记录器
    this->gimbal_adjust_history_.pop_front();
    this->gimbal_adjust_history_.push_back(0);
    //* 检测装甲板记录器
    this->armor_detected_history_.pop_front();
    this->armor_detected_history_.push_back(0);
    //* 检测敌人记录器
    this->enemy_detected_history_.pop_front();
    this->enemy_detected_history_.push_back(0);
    //* 检测敌人front姿势记录器
    this->enemy_front_pose_history_.pop_front();
    this->enemy_front_pose_history_.push_back(0);

    if (robot_detect->detected)
    {
        //* 默认装机版未检测到
        bool enemy_armor_detected = false;
        //* 判断装甲板
        for (roborts_msgs::ArmorMsg armor_msg : robot_detect->detected_info) {
            if (armor_msg.armor_detected) {
                //done 判断敌我
                if (blackboard_ptr->lurk_mode == LurkingStatus::Lurking)
                {
                    //* 潜伏模式 --- 先用ID
                    if(armor_msg.id == this->ID || armor_msg.id +100 == this->ID)
                    {
                        //* 解算敌方位置、对比队友位置
                        geometry_msgs::PoseStamped enemy_armor_map_pose;
                        GetArmorPose(armor_msg.pose, enemy_armor_map_pose);
                        if (!CheckCrossWithHighObstacle(enemy_armor_map_pose,this->my_Pose))
                        {
                            std::cout << "enemy cross Obstacle!" << std::endl;
                            continue;
                        }
                        else 
                        {
                            std::cout << "enemy!" << std::endl;
                        }
                    }
                    else
                    { //* ID不同
                        //* 检查队友位置是否有效
                        if ( ros::Time::now().toSec() - blackboard_ptr->friend_Pose.header.stamp.toSec() > 1 )
                        {
                            continue;
                        }
                        else {
                            //* 解算敌方位置、对比队友位置
                            geometry_msgs::PoseStamped enemy_armor_map_pose;
                            GetArmorPose(armor_msg.pose, enemy_armor_map_pose);
                            //* 60cm之内 判断为队友
                            if (Compute_Distance_BetweenBoth(
                                    enemy_armor_map_pose.pose.position.x,
                                    enemy_armor_map_pose.pose.position.y,
                                    this->friend_Pose.pose.position.x,
                                    this->friend_Pose.pose.position.y) < 0.6)
                            {
                                continue;
                            }
                            else
                            {
                                //* 解算敌方位置、对比队友位置
                                geometry_msgs::PoseStamped enemy_armor_map_pose;
                                GetArmorPose(armor_msg.pose, enemy_armor_map_pose);
                                if (!CheckCrossWithHighObstacle(enemy_armor_map_pose, this->my_Pose))
                                {
                                    std::cout << "enemy cross Obstacle!" << std::endl;
                                    continue;
                                }
                                else 
                                {
                                    std::cout << "enemy!" << std::endl;
                                }
                            }
                        }
                    }
                }
                else 
                {//* 非潜伏**用颜色
                    if (this->ID < 100) 
                    {
                        if(armor_msg.color == 0) {
                            continue;
                        } else {
                            std::cout << "enemy!" << std::endl;
                        }
                    } 
                    else {
                        if (armor_msg.color == 0)
                        {
                            std::cout << "enemy!" << std::endl;
                        }
                        else {
                            continue;
                        }
                    }
                }
                //* 更新敌人检测记录
                this->enemy_detected_history_.pop_front();
                this->enemy_detected_history_.push_back(1);
                //* 根据距离和装甲板面积选取适合打击的装甲板
                distance_current = Compute_Distance_Point(armor_msg.pose.x, armor_msg.pose.y, armor_msg.pose.z);
                if (distance_current < armor_distance && armor_msg.area > area)
                {
                    choosed_armor = armor_msg;
                    armor_distance = distance_current;
                    area = armor_msg.area;
                }
                //* 检测到敌方装甲板
                enemy_armor_detected = true;
            }
        }// for循环结束
        //done * 根据是否检测到装甲板，计算射击相关信息
        if (enemy_armor_detected) {
            //* 更新装甲板记录器
            this->armor_detected_history_.pop_back();
            this->armor_detected_history_.push_back(1);
            //* 检测到装甲板，开始计算距离、射击次数
            //* 选择enemy_pose
            recent_armor_detected_time = ros::Time::now();
            //* 记录检测到的敌人front姿势
            if (choosed_armor.robot_pose == 1)
            {
                this->enemy_front_pose_history_.pop_back();
                this->enemy_front_pose_history_.push_back(1);
            }
            //* 弹道模型，得到云台调整角度值
            double x = choosed_armor.pose.x;
            double y = choosed_armor.pose.y;
            double z = choosed_armor.pose.z;
            double dis = Compute_Distance_Point(x, y);
            if (fabs(x- this->choose_armor_in_camera_.x) + fabs(y-this->choose_armor_in_camera_.y) < this->armor_change_threshold)
            {
                this->shoot_number = 5;
                this->armor_choosed_times = (this->armor_choosed_times >= 2) ? this->armor_choosed_times : (this->armor_choosed_times + 1);
                // if (this->armor_choosed_times > 1) {
                //     cv::Point2d point_predict = this->kf_predictor_p_->KF_Predict(x, y, false);
                //     x = point_predict.x;
                //     y = point_predict.y;
                //     dis = Compute_Distance_Point(x, y);
                // }
            }
            else
            {
                this->shoot_number = 1;
                this->armor_choosed_times = 0;
                this->kf_predictor_p_->KF_Predict(x, y, true);
            }

            //done 距离 //* 检测到敌人装甲板，但是距离大，不适合打击，但是也可以调整云台
            //* 弹道模型获取pitch 和 yaw
            float next_pitch = -GetPitch(dis, -(h + h * (2.7 - dis) * this->h_adjust), this->my_shoot_speed) + this->offset_pitch;
            float next_yaw = (float)(atan2(y + fabs(y + this->offset_y ) * this->offset_y_weight, x)) + this->offset_yaw;

            //* 不需要调整yaw值
            if ( fabs(next_yaw) < yaw_change_threshold ) 
            {
                //* 记录更新  *** yaw 为零 默认云台不会调整，就可以连发子弹 //* 云台稳定
                this->gimbal_adjust_history_.pop_back();
                this->gimbal_adjust_history_.push_back(1);
                next_yaw = 0;
            }
            //* 不需要调整pitch值
            if (fabs(next_pitch - last_pitch_) < pitch_change_threshold)
            {
                next_pitch = last_pitch_;
            }
            //* 更新保留值
            last_pitch_ = next_pitch;
            last_yaw_ = next_yaw;
            //* 更新装甲板面积和敌方距离
            this->enemy_distance = armor_distance;
            this->enemy_armor_area = area;
            //* 更新选择的装甲板信息
            this->choose_armor_in_camera_ = choosed_armor.pose;

            // //* 云台调整消息
            //* 
            //* 云台调整
            this->shoot_control_msg_.pitch_angle = next_pitch;
            if (this->use_pid) {
                const fp32 yaw_PID[3] = {
                    this->pid_kp,
                    this->pid_ki,
                    this->pid_kd
                    };
                PID_init(&(this->yaw_pid), PID_POSITION, yaw_PID, this->max_out, this->max_iout);
                //* pid calculate. PID计算
                PID_calc(&(this->yaw_pid), 0.0, next_yaw);
                this->shoot_control_msg_.yaw_angle = yaw_pid.out;
            } 
            else {
                this->shoot_control_msg_.yaw_angle = next_yaw * this->pinch_param;
            }
            //* 发布控制信息
            this->shoot_control_publisher_.publish(this->shoot_control_msg_);
            this->gimbal_angle_msgs_.yaw_mode = true;
            this->gimbal_angle_msgs_.yaw_angle = this->shoot_control_msg_.yaw_angle;
            this->gimbal_angle_msgs_.pitch_angle = next_pitch;
            //this->gimbal_executor_->Execute(this->gimbal_angle_msgs_);
        }
        else {
            //done 没有检测到装甲板，距离不需要设置为很大
            this->if_enemy_armor_detected = false;
            //this->enemy_distance = 1000.0;
            this->enemy_armor_area = 0.0;
            this->armor_choosed_times = 0;

            // //* 锁， 修改yaw轴相对运动角度为0，pitch默认为上一次的状态
            // {
            //     std::lock_guard<std::mutex> lock_guard(this->mutex_gimbal_angle_);
            //     this->gimbal_yaw_offset = 0;
            // }
        }
    }
    else {
        //done 没检测到敌人立即设置“armor detected”为false；
        //*done 更新敌人检测记录器
        ROS_INFO("No valid armor");
        
        this->gimbal_adjust_history_.shrink_to_fit();
        this->enemy_distance = 1000.0;
        this->enemy_armor_area = 0.0;
        this->armor_choosed_times = 0;
    }

    uint8_t accu = 0;
    // //done 统计敌人检测历史记录器
    accu = std::accumulate(this->enemy_detected_history_.begin(), this->enemy_detected_history_.end(), accu);
    //* 检测到装甲板的次数大于一定阈值--判断敌人是否检测到---
    if (accu >= this->enemy_detected_histoy_threshold_)
    {
        this->if_enemy_detected = true;
    }
    else {
        this->if_enemy_detected = false;
    }
    //done 统计装甲板检测历史记录器
    accu = std::accumulate(this->armor_detected_history_.begin(), this->armor_detected_history_.end(), accu);
    //* 判断装甲板是否检测到
    if (accu >= this->armor_detected_history_threshold_)
    {
        this->if_enemy_armor_detected = true;
    }
    else {
        this->if_enemy_armor_detected = false;
    }

    return;

    //todo 下面没用

    if (!ChooseArmorForShoot(robot_detect->detected_info, choosed_armor, shoot_num))
    {
        ROS_INFO("No valid armor");
        this->shoot_number = 0;
        if ((ros::Time::now() - recent_armor_detected_time).toSec() > 1)
        {
            this->if_enemy_detected = false;
        }
        return;
    }

    recent_armor_detected_time = ros::Time::now();

    this->shoot_number = shoot_num;

    this->choose_armor_in_camera_ = choosed_armor.pose;

    double x = choosed_armor.pose.x;
    double y = choosed_armor.pose.y;
    double z = choosed_armor.pose.z;
    double dis = sqrt(x * x + y * y);

    float next_pitch = -GetPitch(dis, -h, this->my_shoot_speed);
    float next_yaw = (float)(atan2(y, x)); //+ 5 * PI / 180.0;

    if (fabs(next_yaw - last_yaw_) < yaw_change_threshold)
        next_yaw = last_yaw_;
    if (fabs(next_pitch - last_pitch_) < pitch_change_threshold)
        next_pitch = last_pitch_;

    ROS_INFO("armor_x:%.4f, armor_y:%.4f, Pitch: %.4f yaw: %.4f, shoot_num:%d", choosed_armor.pose.x, choosed_armor.pose.y, next_pitch, next_yaw, shoot_num);

    this->enemy_distance = dis;

{
    std::lock_guard<std::mutex> lock_guard(this->mutex_gimbal_angle_);
    this->gimbal_yaw_offset = next_yaw;
    this->gimbal_pitch_offset = next_pitch;
}

    this->if_enemy_detected = true;

    have_last_armor_ = true;
    last_pitch_ = next_pitch, last_yaw_ = next_yaw;
}


//* 回调函数--机器人友军订阅回调函数
void BlackBoard::Sub_Robot_FriendPose_CallBack_(const geometry_msgs::PoseStamped::ConstPtr& robot_friend_pose) {
    this->if_know_friend_pose = true;
    this->friend_Pose = *robot_friend_pose;
}
//* 回调函数--订阅哨岗消息
void BlackBoard::Sub_Guard_Cars_CallBack_(const roborts_msgs::CarMsgs::ConstPtr &car_msgs)
{
    //* 哨岗消息
    this->guard_cars_msg_ = *car_msgs;
}

//* 回调函数
void BlackBoard::Sub_Robot_Odom_CallBack_(const nav_msgs::Odometry::ConstPtr &odom_info){
    this->odom_info_ = *odom_info;
}


//air friction is considered
float BlackBoard::BulletModel(float x, float v, float angle) { //x:m,v:m/s,angle:rad
  float t, y;
  t = (float)((exp(init_k_ * x) - 1) / (init_k_ * v * cos(angle)));
  y = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
  return y;
}

//x:distance , y: height
float BlackBoard::GetPitch(float x, float y, float v) {
  float y_temp, y_actual, dy;
  float a;
  y_temp = y;
  // by iteration
  for (int i = 0; i < 60; i++) {
    a = (float) atan2(y_temp, x);
    y_actual = BulletModel(x, v, a);
    dy = y - y_actual;
    y_temp = y_temp + dy;
    if (fabsf(dy) < 0.00001) {
      break;
    }
    //printf("iteration num %d: angle %f,temp target y:%f,err of y:%f\n",i+1,a*180/3.1415926535,yTemp,dy);
  }
  return a;

}


double BlackBoard::GetNextY(double x, double y, double k) {
  int nextx = (int) (x + 1.01);
  return y + (nextx - x) * k;
}


bool BlackBoard::CheckVerticalLineCross(int x, int y_1, int y_2) {
  int map_height = costmap_ptr_->GetCostMap()->GetSizeYCell();
  if(y_1 > y_2) std::swap(y_1, y_2);
  y_1 = std::max(0, y_1 - bullet_min_cell_dis), y_2 = std::min(y_2 + bullet_min_cell_dis, map_height - 1);
  //iterate over all high obstacles
  for(int i = 0; i < 4; i++) {
    int bl_x = grass_obstacle_info[i][0], bl_y = grass_obstacle_info[i][1], tr_x = grass_obstacle_info[i][0] + grass_obstacle_info[i][2], tr_y = grass_obstacle_info[i][1] + grass_obstacle_info[i][3];
    if(x >= bl_x && x <= tr_x && !(y_2 < bl_y || y_1 > tr_y)) return false;
  }
  return true;
}

bool BlackBoard::CheckCrossWithHighObstacle(geometry_msgs::PoseStamped start,geometry_msgs::PoseStamped end){
  //std::cout << "the cost in(1.9, 2.2): " << (int)(costmap_ptr_->GetCostMap()->GetCost(124, 40)) << "\n";
  unsigned int startx, starty, endx, endy;
  costmap_ptr_->GetCostMap()->World2Map(start.pose.position.x, start.pose.position.y, startx, starty);
  costmap_ptr_->GetCostMap()->World2Map(end.pose.position.x, end.pose.position.y, endx, endy);
  //std::cout << "startx, starty, endx, endy = " << startx << " " << starty << " " << endx << " " << endy << "\n";
  if(startx > endx) std::swap(startx, endx), std::swap(starty, endy);
  if(startx == endx) {
    if(!CheckVerticalLineCross(startx, starty, endy)) return false;
    return true;
  }else {
    double k = (double)((int)endy - (int)starty) / ((int)endx - (int)startx);
    //std::cout << "startx, starty, nexty, k = " << startx << " " << starty << " " <<  (int)(GetNextY(startx + 0.5, starty + 0.5, k)) + (starty < endy ? 1 : 0) << " " << k << "\n";
    //check for first vertical line
    if(!CheckVerticalLineCross(startx, starty, (int)(GetNextY(startx + 0.5, starty + 0.5, k)) + (starty < endy ? 1 : 0) )) return false;
    double prev_y =  GetNextY(startx + 0.5, starty + 0.5, k);
    for(int x_ = startx + 1; x_ < endx; x_++) {
      double next_y = GetNextY(x_, prev_y, k);
      if(!CheckVerticalLineCross(x_, (int)prev_y + (starty > endy ? 1 : 0), (int)next_y + (starty < endy ? 1 : 0))) return false;
      //std::cout << "x_, prev_y, next_y = " << x_ << " " << (int)prev_y << " " <<  (int)next_y + (starty < endy ? 1 : 0) << std::endl;
      prev_y = next_y;
    }
    if(!CheckVerticalLineCross(endx, prev_y, endy)) return false;
    //std::cout << "endx, prev_y, endy = " << endx << " " << (int)prev_y << " " <<  endy << std::endl;
    //std::cout << "\n\n";
    return true;
  }
}


bool BlackBoard::GetArmorPose(geometry_msgs::Point &armor_pose_, geometry_msgs::PoseStamped &global_pose) {
  geometry_msgs::PoseStamped armor_pose;
  armor_pose.pose.position.x = armor_pose_.x;
  armor_pose.pose.position.y = armor_pose_.y;
  armor_pose.pose.orientation.w =  1.0;
  armor_pose.header.frame_id = "camera";
  armor_pose.header.stamp = ros::Time(0);

  try {
    tf_ptr_->transformPose("map", armor_pose, global_pose);
  }
  catch (tf::LookupException &ex) {
    ROS_ERROR("No Transform Error looking up robot pose: %s", ex.what());
    return false;
  }
  catch (tf::ConnectivityException &ex) {
    ROS_ERROR("Connectivity Error looking up robot pose: %s", ex.what());
    return false;
  }
  catch (tf::ExtrapolationException &ex) {
    ROS_ERROR("Extrapolation Error looking up robot pose: %s", ex.what());
    return false;
  }
  return true;
}


bool BlackBoard::ChooseArmorForShoot(std::vector<roborts_msgs::ArmorMsg> msgs, roborts_msgs::ArmorMsg &choosed_armor, int &shoot_num) {
  //debug
  /*double area = -1;
  for(auto armor : msgs) {
    if(armor.color == enermy_color_ && armor.area > area) {
      choosed_armor = armor;
      area = armor.area;
    }
  }
  if(area > min_area_for_resample) {
    shoot_num = 1;
    return true;
  }else return false;*/

  //debug

  //the distance between current armor position and previous armor position
  double dis  = 10000;
  geometry_msgs::PoseStamped armor_pose;
  geometry_msgs::PoseStamped min_dis_pos;

  //exist last armor for shoot
  if(have_last_armor_) {
    for(auto armor : msgs) {
      //if(armor.color != enermy_color_) continue;
      if(GetArmorPose(armor.pose, armor_pose)) {
        geometry_msgs::PoseStamped robort_pose;
        costmap_ptr_ -> GetRobotPose(robort_pose);
        ROS_INFO("have, roborts_pose: %.4f, %.4f armor_pose: %.4f, %.4f", robort_pose.pose.position.x, robort_pose.pose.position.y, armor_pose.pose.position.x, armor_pose.pose.position.y);
        if(!CheckCrossWithHighObstacle(robort_pose, armor_pose)) {
          ROS_INFO("Cross High obstacles.");
          continue;
        }
        // double cur_dis = Distance(last_armor_for_shoot_, armor_pose);
        // if(cur_dis < dis) 
        // {
        //     dis = cur_dis;
        //     min_dis_pos = armor_pose;
        //     choosed_armor = armor;
        // }
      }
    }
    if(dis > min_dis_for_resample || choosed_armor.area < min_area_for_resample) {
      //last_armor_for_shoot_ = min_dis_pos;
      have_last_armor_ = false;
      shoot_num = 0;
      return false;
    }else{
      shoot_num = 4;
      return true;
    }
  }else{
    double optimal_point = -1;
    //find optimal armor
    for(auto &armor : msgs) {
      //if(armor.color != enermy_color_) continue;
      if(GetArmorPose(armor.pose, armor_pose)) {
        geometry_msgs::PoseStamped robort_pose;
        costmap_ptr_ -> GetRobotPose(robort_pose);
        ROS_INFO("dont have, roborts_pose: %.4f, %.4f armor_pose: %.4f, %.4f", robort_pose.pose.position.x, robort_pose.pose.position.y, armor_pose.pose.position.x, armor_pose.pose.position.y);
        if(!CheckCrossWithHighObstacle(robort_pose, armor_pose)) {
          ROS_INFO("Croos High obstacles.");
          continue;
        }
      }else {
        continue;
      }
      double dis = sqrt(armor.pose.x * armor.pose.x + armor.pose.y * armor.pose.y);
      //the points for this armor
      double armor_point = (5 - dis) / 5 + (double)armor.area / 2000;
      if(armor_point > optimal_point) {
        optimal_point = armor_point;
        choosed_armor = armor;
      }
    }
    //whether to choose the optimal armor
    if(optimal_point > 0 && choosed_armor.area > min_area_for_resample) {
      have_last_armor_ = true;
      //GetArmorPose(choosed_armor.pose, last_armor_for_shoot_);
      //ROS_INFO("chooose, armor_pose: %.4f, %.4f", last_armor_for_shoot_.pose.position.x, last_armor_for_shoot_.pose.position.y);

      shoot_num = 1;
      return true;
    }else{
      shoot_num = 0;
      return false;
    }
  }
}

double BlackBoard::Distance(geometry_msgs::PoseStamped &p_1, geometry_msgs::PoseStamped &p_2) {
  return sqrt((p_1.pose.position.x - p_2.pose.position.x) * (p_1.pose.position.x - p_2.pose.position.x) + (p_1.pose.position.y - p_2.pose.position.y) * (p_1.pose.position.y - p_2.pose.position.y));
}


void BlackBoard::StartSpin() {
    ros::spin();
}