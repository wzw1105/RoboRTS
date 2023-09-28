#include "./blackboard.h"

namespace roborts_decision{

BlackBoard::BlackBoard(GimbalExecutor* gimbal) {
    gimbal_executor_ = gimbal;
    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
    std::string map_path = ros::package::getPath("roborts_costmap") + "/config/costmap_parameter_config_for_decision.prototxt";
    costmap_ptr_ = std::make_shared<roborts_costmap::CostmapInterface>("decision_costmap", *tf_ptr_, map_path);
    
    //subscriber
    this->ros_sub_referee_HP_ = this->nh.subscribe("game_robot_hp",3,&BlackBoard::Sub_GameRobotHP_CallBack_,this);  // advertise<roborts_msgs::GameRobotHP>
    this->ros_sub_referee_Bullets_ = this->nh.subscribe("game_robot_bullet",3,&BlackBoard::Sub_GameRobotBullet_CallBack_,this);// advertise<roborts_msgs::GameRobotBullet>("game_robot_bullet",3);
    this->ros_sub_referee_GameZoneArray_ = this->nh.subscribe("game_zone_array_status",3,&BlackBoard::Sub_GameZoneArrayStatus_CallBack_,this);// advertise<roborts_msgs::GameZoneArray>("game_zone_array_status",3);
    this->ros_sub_referee_GameStatus_ = this->nh.subscribe("game_status",3,&BlackBoard::Sub_GameStatus_CallBack_,this);// advertise<roborts_msgs::GameStatus>("game_status",3);
    this->ros_sub_referee_LurkStatus_ = this->nh.subscribe("game_lurk_status",3,&BlackBoard::Sub_GameLurkStatus_CallBack_,this);// advertise<roborts_msgs::LurkStatus>("game_lurk_status",3);
    this->ros_sub_referee_Robot_status_ = this->nh.subscribe("robot_status", 3, &BlackBoard::Sub_RobotStatus_CallBack_, this);
    this->ros_sub_referee_Robot_damage_ = this->nh.subscribe("robot_damage", 3, &BlackBoard::Sub_RobotDamage_CallBack_, this);
    this->ros_sub_referee_Robot_heat_ = this->nh.subscribe("robot_heat", 3, &BlackBoard::Sub_RobotHeat_CallBack_, this);
    this->ros_sub_referee_Shoot_info_= this->nh.subscribe("robot_shoot", 3, &BlackBoard::Sub_RobotShootInfo_CallBack_, this);
    this->ros_sub_robot_pose_ = this->nh.subscribe("amcl_pose", 3, &BlackBoard::Sub_RobotPose_CallBack_, this);
    this->ros_sub_robot_detect_ = this->nh.subscribe("armors_info", 1, &BlackBoard::Sub_RobotDetectEnemy_CallBack_, this);
    this->ros_sub_robot_friendpose_ = this->nh.subscribe("friend_pose", 3, &BlackBoard::Sub_Robot_FriendPose_CallBack_, this);
    this->ros_sub_robot_odom_ = this->nh.subscribe("odom", 1, &BlackBoard::Sub_Robot_Odom_CallBack_, this);
    this->ros_sub_guard_msg_ = this->nh.subscribe("pose_from_stamp", 1, &BlackBoard::Sub_Guard_Msg_CallBack_, this);
    this->shoot_client = nh.serviceClient<roborts_msgs::ShootCmd>("cmd_shoot");
    this->fric_client = nh.serviceClient<roborts_msgs::FricWhl>("cmd_fric_wheel");

    //F1 ~ F6 区域
    this->F_1_6[0].header.frame_id = "map";
    this->F_1_6[0].pose.position.x = 7.58;
    this->F_1_6[0].pose.position.y = 1.69;
    this->F_1_6[0].pose.position.z = 0;
    this->F_1_6[0].pose.orientation = tf::createQuaternionMsgFromYaw(M_PI);
    this->F_1_6[1].header.frame_id = "map";
    this->F_1_6[1].pose.position.x = 6.18;
    this->F_1_6[1].pose.position.y = 2.83;
    this->F_1_6[1].pose.position.z = 0;
    this->F_1_6[1].pose.orientation = tf::createQuaternionMsgFromYaw(0);
    this->F_1_6[2].header.frame_id = "map";
    this->F_1_6[2].pose.position.x = 4.04;
    this->F_1_6[2].pose.position.y = 0.445;
    this->F_1_6[2].pose.position.z = 0;
    this->F_1_6[2].pose.orientation = tf::createQuaternionMsgFromYaw(0);
    this->F_1_6[3].header.frame_id = "map";
    this->F_1_6[3].pose.position.x = 4.04;
    this->F_1_6[3].pose.position.y = 4.035;
    this->F_1_6[3].pose.position.z = 0;
    this->F_1_6[3].pose.orientation = tf::createQuaternionMsgFromYaw(0);
    this->F_1_6[4].header.frame_id = "map";
    this->F_1_6[4].pose.position.x = 1.9;
    this->F_1_6[4].pose.position.y = 1.65;
    this->F_1_6[4].pose.position.z = 0;
    this->F_1_6[4].pose.orientation = tf::createQuaternionMsgFromYaw(0);
    this->F_1_6[5].header.frame_id = "map";
    this->F_1_6[5].pose.position.x = 0.5;
    this->F_1_6[5].pose.position.y = 2.79;
    this->F_1_6[5].pose.position.z = 0;
    this->F_1_6[5].pose.orientation = tf::createQuaternionMsgFromYaw(0);

    //* default_patrol_goal_array 
    //car1
    this->default_patrol_goal_array[0][0].pose.position.x = 6;
    this->default_patrol_goal_array[0][0].pose.position.y = 1;
    this->default_patrol_goal_array[0][0].pose.position.z = 0;
    this->default_patrol_goal_array[0][0].pose.orientation = tf::createQuaternionMsgFromYaw(-0.4);
    this->default_patrol_goal_array[0][1].pose.position.x = 7.5;
    this->default_patrol_goal_array[0][1].pose.position.y = 2.0;
    this->default_patrol_goal_array[0][1].pose.position.z = 0;
    this->default_patrol_goal_array[0][1].pose.orientation = tf::createQuaternionMsgFromYaw(1.57);
    this->default_patrol_goal_array[0][2].pose.position.x = 2.0;
    this->default_patrol_goal_array[0][2].pose.position.y = 3.4;
    this->default_patrol_goal_array[0][2].pose.position.z = 0;
    this->default_patrol_goal_array[0][2].pose.orientation = tf::createQuaternionMsgFromYaw(2.74);
    this->default_patrol_goal_array[0][3].pose.position.x = 0.75;
    this->default_patrol_goal_array[0][3].pose.position.y = 2.0;
    this->default_patrol_goal_array[0][3].pose.position.z = 0;
    this->default_patrol_goal_array[0][3].pose.orientation = tf::createQuaternionMsgFromYaw(-1.57);
    
    //car2
    this->default_patrol_goal_array[1][0].pose.position.x = 4.6;
    this->default_patrol_goal_array[1][0].pose.position.y = 2.8;
    this->default_patrol_goal_array[1][0].pose.position.z = 0;
    this->default_patrol_goal_array[1][0].pose.orientation = tf::createQuaternionMsgFromYaw(0.4);
    this->default_patrol_goal_array[1][1].pose.position.x = 4.6;
    this->default_patrol_goal_array[1][1].pose.position.y = 2.8;
    this->default_patrol_goal_array[1][1].pose.position.z = 0;
    this->default_patrol_goal_array[1][1].pose.orientation = tf::createQuaternionMsgFromYaw(-1.17);
    this->default_patrol_goal_array[1][2].pose.position.x = 3.4;
    this->default_patrol_goal_array[1][2].pose.position.y = 2.8;
    this->default_patrol_goal_array[1][2].pose.position.z = 0;
    this->default_patrol_goal_array[1][2].pose.orientation = tf::createQuaternionMsgFromYaw(M_PI);
    this->default_patrol_goal_array[1][3].pose.position.x = 3.4;
    this->default_patrol_goal_array[1][3].pose.position.y = 1.6;
    this->default_patrol_goal_array[1][3].pose.position.z = 0;
    this->default_patrol_goal_array[1][3].pose.orientation = tf::createQuaternionMsgFromYaw(-1.17);
    this->default_patrol_goal_array[1][4].pose.position.x = 3.4;
    this->default_patrol_goal_array[1][4].pose.position.y = 1.6;
    this->default_patrol_goal_array[1][4].pose.position.z = 0;
    this->default_patrol_goal_array[1][4].pose.orientation = tf::createQuaternionMsgFromYaw(0.4);
    this->default_patrol_goal_array[1][5].pose.position.x = 4.6;
    this->default_patrol_goal_array[1][5].pose.position.y = 1.6;
    this->default_patrol_goal_array[1][5].pose.position.z = 0;
    this->default_patrol_goal_array[1][5].pose.orientation = tf::createQuaternionMsgFromYaw(0);

    //friend dead
    this->default_patrol_goal_array[2][0].pose.position.x = 6;
    this->default_patrol_goal_array[2][0].pose.position.y = 1;
    this->default_patrol_goal_array[2][0].pose.position.z = 0;
    this->default_patrol_goal_array[2][0].pose.orientation = tf::createQuaternionMsgFromYaw(-0.4);
    this->default_patrol_goal_array[2][1].pose.position.x = 6;
    this->default_patrol_goal_array[2][1].pose.position.y = 1;
    this->default_patrol_goal_array[2][1].pose.position.z = 0;
    this->default_patrol_goal_array[2][1].pose.orientation = tf::createQuaternionMsgFromYaw(1.17);
    this->default_patrol_goal_array[2][2].pose.position.x = 7.5;
    this->default_patrol_goal_array[2][2].pose.position.y = 2.0;
    this->default_patrol_goal_array[2][2].pose.position.z = 0;
    this->default_patrol_goal_array[2][2].pose.orientation = tf::createQuaternionMsgFromYaw(1.57);
    this->default_patrol_goal_array[2][3].pose.position.x = 6.0;
    this->default_patrol_goal_array[2][3].pose.position.y = 2.8;
    this->default_patrol_goal_array[2][3].pose.position.z = 0;
    this->default_patrol_goal_array[2][3].pose.orientation = tf::createQuaternionMsgFromYaw(-1.17);
    this->default_patrol_goal_array[2][4].pose.position.x = 2.0;
    this->default_patrol_goal_array[2][4].pose.position.y = 3.4;
    this->default_patrol_goal_array[2][4].pose.position.z = 0;
    this->default_patrol_goal_array[2][4].pose.orientation = tf::createQuaternionMsgFromYaw(1.17);
    this->default_patrol_goal_array[2][5].pose.position.x = 2.0;
    this->default_patrol_goal_array[2][5].pose.position.y = 3.4;
    this->default_patrol_goal_array[2][5].pose.position.z = 0;
    this->default_patrol_goal_array[2][5].pose.orientation = tf::createQuaternionMsgFromYaw(-0.4);
    this->default_patrol_goal_array[2][6].pose.position.x = 0.75;
    this->default_patrol_goal_array[2][6].pose.position.y = 2.0;
    this->default_patrol_goal_array[2][6].pose.position.z = 0;
    this->default_patrol_goal_array[2][6].pose.orientation = tf::createQuaternionMsgFromYaw(-1.57);
    this->default_patrol_goal_array[2][7].pose.position.x = 2.2;
    this->default_patrol_goal_array[2][7].pose.position.y = 1.5;
    this->default_patrol_goal_array[2][7].pose.position.z = 0;
    this->default_patrol_goal_array[2][7].pose.orientation = tf::createQuaternionMsgFromYaw(0.4);

    friend_pose_time = ros::Time::now() - ros::Duration(600);
    recent_armor_damage_time = ros::Time::now() - ros::Duration(600); 
    recent_enemy_detect_time = ros::Time::now() - ros::Duration(600); 
    cycle_enermy_detect_time[total_index - 1] = ros::Time::now() - ros::Duration(600);
    for(int i = 0; i < total_index; i++) choose_armor[i] = false;
    for(int i = 0; i < total_hp_index; i++) self_recent_hp[i] = 2000;

    enemy_area_goal.header.frame_id = "map";
}

//? 机器人自身状态订阅回调函数
void BlackBoard::Sub_RobotStatus_CallBack_(const roborts_msgs::RobotStatus::ConstPtr& robot_status) {
    //* ID、血量、底盘/云台/枪口是否可操控
    this->self_remain_hp = robot_status->remain_hp;
    this->self_id = (robot_status -> id % 100) - 1;
    this->self_color = robot_status -> id < 10 ? 0 : 1; //红：0 蓝：1

    //* 底盘、云台、枪管是否可用
    this->gimbal_enable = robot_status->gimbal_enable;
    this->chassis_enable = robot_status->chassis_enable;
    this->shooter_enable = robot_status->shooter_enable;

    //check to open fric wheel
    if(! this -> last_shoot_enable && this -> shooter_enable) {
        ros::ServiceClient   fric_client = nh.serviceClient<roborts_msgs::FricWhl>("cmd_fric_wheel");
        roborts_msgs::FricWhl fricCtrl;
        fricCtrl.request.open = true;
        if(fric_client.call(fricCtrl)) {
            this -> last_shoot_enable = true;
            ROS_INFO("Open Fric successfully.");
        }else{
            ROS_INFO("Open Failed.");
        }
    }else{
        this -> last_shoot_enable = this -> shooter_enable;
    }

    //* 热量冷却、限制
    this->heat_cooling_rate = robot_status->heat_cooling_rate;
    this->heat_cooling_limit = robot_status->heat_cooling_limit;

    if(init_data_with_robot_id) return;

    // 初始化启动区位置
    if(this -> self_id == 0) {
        boot_area_pose.header.frame_id = "map";
        boot_area_pose.pose.position.x = 0.5;
        boot_area_pose.pose.position.y = 0.5;
        boot_area_pose.pose.position.z = 0;
        boot_area_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    } else{
        boot_area_pose.header.frame_id = "map";
        boot_area_pose.pose.position.x = 0.5;
        boot_area_pose.pose.position.y = 3.98;
        boot_area_pose.pose.position.z = 0;
        boot_area_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);    
    }

    init_data_with_robot_id = true;
}

void BlackBoard::Sub_GameRobotHP_CallBack_(const roborts_msgs::GameRobotHP::ConstPtr &game_robot_hp){
    this -> robot_hp[0][0] = game_robot_hp -> red1;
    this -> robot_hp[0][1] = game_robot_hp -> red2;
    this -> robot_hp[1][0] = game_robot_hp -> blue1;
    this -> robot_hp[1][1] = game_robot_hp -> blue2;

    if(this -> self_color != -1 && this -> robot_hp[this -> self_color][(this -> self_id + 1) % 2] == 0) this -> friend_dead = true;
}

void BlackBoard::Sub_GameRobotBullet_CallBack_(const roborts_msgs::GameRobotBullet::ConstPtr &game_robot_bullet){
    this -> robot_bullets[0][0] = game_robot_bullet -> red1;
    this -> robot_bullets[0][1] = game_robot_bullet -> red2;
    this -> robot_bullets[1][0] = game_robot_bullet -> blue1;
    this -> robot_bullets[1][1] = game_robot_bullet -> blue2;
}

/**
    RED_HP_RECOVERY=1
    RED_BULLET_SUPPLY=2
    BLUE_HP_RECOVERY=3
    BLUE_BULLET_SUPPLY=4
    DISABLE_SHOOTING=5
    DISABLE_MOVEMENT=6
*/
void BlackBoard::Sub_GameZoneArrayStatus_CallBack_(const roborts_msgs::GameZoneArray::ConstPtr& game_zone_array_status) {
    this -> game_zone_array = * game_zone_array_status;
    if(this -> self_color == -1) return;

    for(int i = 0; i < 6; i++) {
        //红方
        if(this -> self_color == 0 && game_zone_array_status->zone[i].type == 1) {
            blood_add_position_id = i;
            blood_add_area_active = game_zone_array_status->zone[i].active;
        }
        if(this -> self_color == 0 && game_zone_array_status->zone[i].type == 2) {
            bullets_add_position_id = i;
            bullets_add_area_active = game_zone_array_status->zone[i].active;
        }
        //蓝方
        if(this -> self_color == 1 && game_zone_array_status->zone[i].type == 3) {
            blood_add_position_id = 5 - i;
            blood_add_area_active = game_zone_array_status->zone[i].active;
        }
        if(this -> self_color == 1 && game_zone_array_status->zone[i].type == 4) {
            bullets_add_position_id = 5 - i;
            bullets_add_area_active = game_zone_array_status->zone[i].active;
        }
    }
}

void BlackBoard::Sub_GameStatus_CallBack_(const roborts_msgs::GameStatus::ConstPtr& game_status_) {
    this -> remaining_time = game_status_ -> remaining_time;
    this -> game_status = game_status_ -> game_status;
}

void BlackBoard::Sub_GameLurkStatus_CallBack_(const roborts_msgs::LurkStatus::ConstPtr &lurk_status_){
    this -> lurk_status = lurk_status_ -> lurk_mode;
}

void BlackBoard::Sub_RobotDamage_CallBack_(const roborts_msgs::RobotDamage::ConstPtr& robot_damage) {
    this->robot_damage = *robot_damage;
    if (robot_damage -> damage_type == 0) {
        recent_armor_damage_time = ros::Time::now();
        recent_armor_damege_id;
    }
}

void BlackBoard::Sub_RobotHeat_CallBack_(const roborts_msgs::RobotHeat::ConstPtr& robot_heat) {
    this -> shooter_heat = robot_heat -> shooter_heat;
}

void BlackBoard::Sub_RobotShootInfo_CallBack_(const roborts_msgs::RobotShoot::ConstPtr& robot_shoot) {
    this -> shoot_frequency = robot_shoot -> frequency;
    this -> shoot_speed = robot_shoot -> speed;
}

void BlackBoard::Sub_RobotPose_CallBack_(const geometry_msgs::PoseStamped::ConstPtr& robot_pose) {
    this -> self_pose = * robot_pose;
}

//callback实时对装甲板信息进行处理，计算弹道放到行为节点，选取的装甲板信息以及发弹量存储到cur_hit_armor和shoot_num里
void BlackBoard::Sub_RobotDetectEnemy_CallBack_(const roborts_msgs::ArmorMsgs::ConstPtr& msgs) {
    if(self_color == -1 || self_id == -1) return;
    if (!msgs -> detected) {
        ros::Time now_ = ros::Time::now();
        choose_armor[cur_armor_index] = false; //记录未检测到
        cycle_enermy_detect_time[cur_armor_index] = now_;
        cur_armor_index = (cur_armor_index + 1) % total_index; //更新index
        {
            std::lock_guard<std::mutex> shoot_lock(shoot_mtx_);
            this_frame_detected = false;
            shoot_num = 0;
        }
        return;
    }
    if(!ChooseArmorForShoot(msgs -> detected_info, this -> cur_hit_armor, this -> shoot_num)) {
        //ROS_INFO("No valid armor for shoot.");
        return;
    } 
    std::cout << "choosed Armor : " << this -> cur_hit_armor.pose.x << " " << this -> cur_hit_armor.pose.y << "\n";
}

//* 毁掉函数--机器人友军威姿订阅回调函数
void BlackBoard::Sub_Robot_FriendPose_CallBack_(const geometry_msgs::PoseStamped::ConstPtr& robot_friend_pose) {
    this -> friend_Pose = * robot_friend_pose;
    this -> friend_pose_time = robot_friend_pose -> header.stamp;
}

void BlackBoard::Sub_Robot_Odom_CallBack_(const nav_msgs::Odometry::ConstPtr &odom_info){
    this->odom_info_ = *odom_info;
}

void BlackBoard::Sub_Guard_Msg_CallBack_(const roborts_msgs::CarMsgs::ConstPtr& guard_msgs) {
    this -> guard_msgs = *guard_msgs;
}

//----------------------- 功能区 -------------------------------

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
  unsigned int startx, starty, endx, endy;
  costmap_ptr_->GetCostMap()->World2Map(start.pose.position.x, start.pose.position.y, startx, starty);
  costmap_ptr_->GetCostMap()->World2Map(end.pose.position.x, end.pose.position.y, endx, endy);
  if(startx > endx) std::swap(startx, endx), std::swap(starty, endy);
  if(startx == endx) {
    if(!CheckVerticalLineCross(startx, starty, endy)) return false;
    return true;
  }else {
    double k = (double)((int)endy - (int)starty) / ((int)endx - (int)startx);
    if(!CheckVerticalLineCross(startx, starty, (int)(GetNextY(startx + 0.5, starty + 0.5, k)) + (starty < endy ? 1 : 0) )) return false;
    double prev_y =  GetNextY(startx + 0.5, starty + 0.5, k);
    for(int x_ = startx + 1; x_ < endx; x_++) {
      double next_y = GetNextY(x_, prev_y, k);
      if(!CheckVerticalLineCross(x_, (int)prev_y + (starty > endy ? 1 : 0), (int)next_y + (starty < endy ? 1 : 0))) return false;
      prev_y = next_y;
    }
    if(!CheckVerticalLineCross(endx, prev_y, endy)) return false;
    return true;
  }
}


bool BlackBoard::CheckArmorCanShoot(roborts_msgs::ArmorMsg &armor, geometry_msgs::PoseStamped &armor_pose){ //根据比赛阶段的不同（前两分钟和后一分钟）检查该装甲板是否可以打击（依据颜色和ID和友军位姿进行判断）
    ros::Time now_ = ros::Time::now();
    //公共过滤方法
    if(!CheckCrossWithHighObstacle(this -> self_pose, armor_pose)) { //检查自己的位置和目标位置是否穿过玻璃
        ROS_INFO("Cross High obstacles.");
        return false;
    }
    double self_enemy_dis = sqrt(armor.pose.x * armor.pose.x + armor.pose.y * armor.pose.y);
    if(self_enemy_dis > max_dis_for_shoot) return false; //太远或面积太小
    
    //正常阶段和潜伏阶段具有不同的过滤方法
    if(this -> lurk_status < 2) { //正常阶段和准备阶段
        if(armor.color == this -> self_color) return false; //自己人
        return true;
    }else{//潜伏阶段
        if(armor.id != this -> self_id) {
            if(now_.toSec() - friend_pose_time.toSec() < 0.5 && Distance(friend_Pose, armor_pose) > lurk_status_danger_shoot_distance) return true; //队友0.5s内位置信息可用且装甲板与之距离小于2m
            return false;
        }else{ //ID相同一定是敌人
            return true;
        }
    }
}
//重新寻找一个距离合适、面积合适的最优装甲板（综合距离、装甲面积、装甲板位置考虑）
bool BlackBoard::ReFindOptimalArmorForShoot(std::vector<roborts_msgs::ArmorMsg> &msgs, roborts_msgs::ArmorMsg &decision_armor, ros::Time &now_) {
    double max_armor_score = -1000;
    roborts_msgs::ArmorMsg max_score_armor;
    geometry_msgs::PoseStamped choosed_armor_pose; //与上一次装甲板最近的装甲板的map坐标
    geometry_msgs::PoseStamped armor_pose;

    for(auto &armor : msgs) {
        //检查是否适合打击
        armor_pose = this -> self_pose;
        GetArmorMapPose(armor.pose, armor_pose);
        if(!CheckArmorCanShoot(armor, armor_pose)) continue; //check这个装甲板是否可以打击
        
        double armor_score = CalculateArmorScore(armor);
        if(armor_score > max_armor_score) {
            choosed_armor_pose = armor_pose;
            max_armor_score = armor_score;
            max_score_armor = armor;
        }
    }
    if(max_armor_score > -1000) {
        ROS_INFO("Newly hittable armor detected !!!");
        choose_armor[cur_armor_index] = true; //记录检测到可打击装甲板
        choosed_armor[cur_armor_index] = max_score_armor;
        cycle_enermy_detect_time[cur_armor_index] = now_;

        cycle_enemy_detect_map_position[cur_armor_index] = choosed_armor_pose;
        cur_armor_index = (cur_armor_index + 1) % total_index; //更新index

        shoot_num = 1;
        return true; //标记检测到敌人
    }else{
        choose_armor[cur_armor_index] = false; //记录未检测到
        cycle_enermy_detect_time[cur_armor_index] = now_;
        cur_armor_index = (cur_armor_index + 1) % total_index; //更新index
        {
            std::lock_guard<std::mutex> shoot_lock(shoot_mtx_);
            this_frame_detected = false;
            shoot_num = 0;
        }
        return false;
    }
}

bool BlackBoard::ChooseArmorForShoot(std::vector<roborts_msgs::ArmorMsg> msgs, roborts_msgs::ArmorMsg &decision_armor, int &shoot_num) {
    ros::Time now_ = ros::Time::now();

    bool have_last_armor_ = false; //距离现在1s内是否存在选过的装甲板
    int last_armor_index = -1; //时间上最近的选取的装甲板的map坐标系信息, 用于找到上一个打击的装甲板
    int first_armor_index = -1; //距离现在1s内的第一个选取过的装甲板，用于判断地方是否在运动，不使用最近的一个的原因在于时间太短，车子即使在运动其位置也变化不大。

    int total_in_1_second = 0, detect = 0;
    std::cout << "Time " << now_.toSec() << " ";
    for(int i = 1; i < total_index; i++) {
        int pre_index = (cur_armor_index - i + total_index) % total_index; //对应的循环数组下标
        if(now_.toSec() - cycle_enermy_detect_time[pre_index].toSec() > 1.0) break; //只考虑1s内
        //total_in_1_second += 1;
        std::cout << " " << now.toSec();
        if(choose_armor[pre_index]) {
            if(!have_last_armor_){ //记录最近的一个
                have_last_armor_ = true;
                last_armor_index = pre_index;
            }
            first_armor_index = pre_index; //记录第一个
            detect += 1;
        }
    }

    std::cout << "\n\n";
    if(detect >= 3) detected_enermy = true;
    else detected_enermy = false;

    //1s内最早的的可打击装甲板检测时间距离现在的时间已经太短，说明刚检测到，无法判断其是否在运动

    double last_cur_armor_dis  = 10000; //计算当前所有装甲板和最近一次选取的装甲板之间的距离的最小值
    geometry_msgs::PoseStamped choosed_armor_pose; //与上一次装甲板最近的装甲板的map坐标
    geometry_msgs::PoseStamped armor_map_pose;
    geometry_msgs::PoseStamped armor_base_link_pose;
  
    //第一步：选取装甲板
    if(have_last_armor_) { 
        //存在最近一次选取的装甲板, 则first和last都不为-1, 

        roborts_msgs::ArmorMsg nearest_armor; //记录与上一次打击的装甲板最近的装甲板
        for(auto armor : msgs) { //通过装甲板的map坐标选取与之最近的装甲板，从而找到上一次打击的装甲板
            GetArmorMapPose(armor.pose, armor_map_pose);
            GetArmorMapPose(armor.pose, armor_base_link_pose);
           
            if(!CheckArmorCanShoot(armor, armor_map_pose)) continue; //check这个装甲板是否可以打击

            //ROS_INFO("have last, roborts_pose: %.4f, %.4f armor_pose: %.4f, %.4f", this -> self_pose.pose.position.x, this -> self_pose.pose.position.y, armor_pose.pose.position.x, armor_pose.pose.position.y);
            //寻找上一次打击的装甲板
            double cur_dis = Distance(cycle_enemy_detect_map_position[last_armor_index], armor_pose);
            if(cur_dis < last_cur_armor_dis) {last_cur_armor_dis = cur_dis; choosed_armor_pose = armor_base_link_pose; nearest_armor = armor;}
        }
        //当前未找到可打击的敌人
        if(last_cur_armor_dis > 9999.0) { 
            choose_armor[cur_armor_index] = false; //记录未检测到
            cycle_enermy_detect_time[cur_armor_index] = now_;
            cur_armor_index = (cur_armor_index + 1) % total_index; //更新index
            shoot_num = 0;
            return false;
        }
        //计算和1s内的第一个选取的装甲板位置的距离
        double first_cur_armor_dis = Distance(choosed_armor_pose, cycle_enemy_detect_map_position[first_armor_index]);
        //视为静止状态
        if(first_cur_armor_dis < no_move_pos_threshold) {
            choose_armor[cur_armor_index] = true; //记录检测到可打击装甲板
            choosed_armor[cur_armor_index] = nearest_armor;
            cycle_enermy_detect_time[cur_armor_index] = now_;
            cycle_enemy_detect_map_position[cur_armor_index] = choosed_armor_pose;
            cur_armor_index = (cur_armor_index + 1) % total_index; //更新index

            double time_ = now_.toSec() - cycle_enermy_detect_time[first_armor_index].toSec(); //第一次检测到该装不动的甲板时距离现在时间很长说明敌人长时间没动，则加大弹量打击，时间短敌人有可能在运动。

            shoot_num = time_ < 0.2 ?  0 : (time_ < 0.4 ? 1 : (time_ < 0.8 ? 4 : 8));
            
            float x = nearest_armor.pose.x, y = nearest_armor.pose.y;
            float dis = sqrt(x * x + y * y);
            float next_pitch = -GetPitch(dis, -h , shoot_speed);
            float next_yaw = (float) (atan2(y, x));
            PublishPitchYawMsgs(next_yaw, next_pitch);

            ROS_INFO("Stand still enemy detected, shoot_num : %d !!!", shoot_num);
            return true;
        //当前装甲板正在运动, 重新选取
        }else{ 
            if(ReFindOptimalArmorForShoot(msgs, decision_armor, now_)) return true;
            return false;
        }
    }else{ //近1s内未选择到过可打击的装甲板，则根据视野中装甲板的得分进行选择，这里不做打击，原因在于不确定敌方运动状态，工作只是为下一次选取装甲板奠定基础
        if(ReFindOptimalArmorForShoot(msgs, decision_armor, now_)) return true;
        return false;
    }
}

bool BlackBoard::GetArmorMapPose(geometry_msgs::Point &armor_pose_, geometry_msgs::PoseStamped &global_pose) {
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

bool BlackBoard::GetArmorBaseLinkPose(geometry_msgs::Point &armor_pose_, geometry_msgs::PoseStamped &global_pose) {
  geometry_msgs::PoseStamped armor_pose;
  armor_pose.pose.position.x = armor_pose_.x;
  armor_pose.pose.position.y = armor_pose_.y;
  armor_pose.pose.orientation.w =  1.0;
  armor_pose.header.frame_id = "camera";
  armor_pose.header.stamp = ros::Time(0);

  try {
    tf_ptr_->transformPose("base_link", armor_pose, global_pose);
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

double BlackBoard::Distance(geometry_msgs::PoseStamped &p_1, geometry_msgs::PoseStamped &p_2) {
  return sqrt((p_1.pose.position.x - p_2.pose.position.x) * (p_1.pose.position.x - p_2.pose.position.x) + (p_1.pose.position.y - p_2.pose.position.y) * (p_1.pose.position.y - p_2.pose.position.y));
}

double BlackBoard::CalculateArmorScore(roborts_msgs::ArmorMsg &armor) {
    double self_enemy_dis = sqrt(armor.pose.x * armor.pose.x + armor.pose.y * armor.pose.y);
    // 距离优势 + 装甲面积优势 + 装甲板扣血量优势
    return (3.0 - self_enemy_dis) / 5 + armor.area / 2000.0 + (armor.robot_pose == 1 ? 1 : (armor.robot_pose == 0 ? 1.2 : ((armor.robot_pose == 2 || armor.robot_pose ==3) ? 1.5 : 2))); 
}

void BlackBoard::PublishPitchYawMsgs(float &next_yaw, float &next_pitch) {
    if(fabs(next_yaw) < yaw_change_threshold) next_yaw = 0;
    if(fabs(next_pitch - cur_pitch_) < pitch_change_threshold) next_pitch = cur_pitch_;
    cur_pitch_ = next_pitch;
    std::cout << "Pitch: " << next_pitch << " Yaw: " << next_yaw << "\n";
    roborts_msgs::GimbalAngle gimbal_cmd_info;

    float tmp_yaw = 0, total_yaw = 0;
    ros::Rate loop_rate(20);
    while(fabs(total_yaw - next_yaw) > 0.001) {
        tmp_yaw = (next_yaw < 0 ? -1 : 1) * yaw_change_unit_;
        total_yaw += tmp_yaw;
        if(next_yaw > 0) {
            if(total_yaw > next_yaw) {
                tmp_yaw -= (total_yaw - next_yaw);
                total_yaw = next_yaw;
            }
        }
        else {
            if(total_yaw < next_yaw) {
                tmp_yaw += (next_yaw - total_yaw);
                total_yaw = next_yaw;
            }
        }

        gimbal_cmd_info.yaw_mode = true;
        gimbal_cmd_info.pitch_mode = false;
        gimbal_cmd_info.yaw_angle = tmp_yaw;
        gimbal_cmd_info.pitch_angle = next_pitch;
        gimbal_executor_ -> Execute(gimbal_cmd_info);
        loop_rate.sleep();
    }
}

//x:distance , y: height
float BlackBoard::GetPitch(float x, float y, float v) {
    float y_temp, y_actual, dy;
    float a;
    y_temp = y;
    // by iteration
    for (int i = 0; i < 50; i++) {
        a = (float) atan2(y_temp, x);
        y_actual = BulletModel(x, v, a);
        dy = y - y_actual;
        y_temp = y_temp + dy;
        if (fabsf(dy) < 0.000001) {
            break;
        }
        //printf("iteration num %d: angle %f,temp target y:%f,err of y:%f\n",i+1,a*180/3.1415926535,yTemp,dy);
    }
}

float BlackBoard::BulletModel(float x, float v, float angle) { //x:m,v:m/s,angle:rad
    float t, y;
    t = (float)((exp(init_k_ * x) - 1) / (init_k_ * v * cos(angle)));
    y = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
    return y;
}

}