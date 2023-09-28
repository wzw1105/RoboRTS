/*
 * @Author: your name
 * @Date: 2022-03-25 15:36:48
 * @LastEditTime: 2022-05-21 22:42:30
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: \roborts_ws\src\RoboRTS\test_pkg\src\Functions.cpp
 */

#include "./Functions.h"


BT::NodeStatus ComputeShootNumber::tick()
{
    if (blackboard_ptr->shooter_heat > 2000){
        setOutput("shoot_number", 0);
    } else if (blackboard_ptr->shooter_heat > 150) {
        setOutput("shoot_number", 1);
    } else {
        //* 如果满足连续几帧检测到敌方车不动
        uint8_t number;
        number = std::accumulate(blackboard_ptr->gimbal_adjust_history_.begin(), blackboard_ptr->gimbal_adjust_history_.end(), number);
        if (number == blackboard_ptr->gimbal_adjust_history_range_) {
            number = Compute_Shoot_Number(blackboard_ptr->shooter_heat, blackboard_ptr->enemy_distance, blackboard_ptr->enemy_armor_area);
        } 
        else {
            setOutput("shoot_number", blackboard_ptr->shoot_number);
        }
    }
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList ComputeShootNumber::providedPorts()
{
    return {BT::OutputPort<uint8_t>("shoot_number")};
}


BT::NodeStatus Choose_Shoot_Enemy() {
    double distance = 100000;
    double distance_compute = 100000;
    
    if (!blackboard_ptr->if_enemy_detected)
        return BT::NodeStatus::FAILURE;

    if (blackboard_ptr->if_enemy_detected) {
        //* 检测到敌人
        for (uint8_t i = 0; i < 2; i++) {
            if (blackboard_ptr->enemies[i]->if_detected) {
                distance_compute
                    = Compute_Distance_Point(blackboard_ptr->enemies[i]->enemy_camera_pose.x,
                        blackboard_ptr->enemies[i]->enemy_camera_pose.y,
                        blackboard_ptr->enemies[i]->enemy_camera_pose.z);
                if (distance_compute < distance) {
                    distance = distance_compute;
                    blackboard_ptr->enemy_suitable_shoot = blackboard_ptr->enemies[i];
                }
            }
        }
        if (distance > 15) {
            return BT::NodeStatus::FAILURE;
        }
    }
}

/**
 * ? 行为节点 ActionNode
 * ? 计算攻击敌人的移动目的地
 * ! 选取最佳击打位置
 */
BT::NodeStatus Compute_Fight_MoveGoal() {

    std::cout << "compute fight move goal" << std::endl;
    
    geometry_msgs::PoseStamped armor_pose;
    geometry_msgs::PoseStamped enemy_armor_pose;
    armor_pose.pose.position.x = blackboard_ptr->choose_armor_in_camera_.x;
    armor_pose.pose.position.y = blackboard_ptr->choose_armor_in_camera_.y;
    armor_pose.pose.orientation.w = 1.0;
    armor_pose.header.frame_id = "camera";
    armor_pose.header.stamp = ros::Time(0);

    try 
    {
        blackboard_ptr->tf_ptr_->transformPose("map", armor_pose, enemy_armor_pose);
        auto dx = enemy_armor_pose.pose.position.x - blackboard_ptr->my_Pose.pose.position.x;
        auto dy = enemy_armor_pose.pose.position.y - blackboard_ptr->my_Pose.pose.position.y;
        auto yaw = std::atan2(dy, dx);
        if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) >= 1.0 && std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 2.0)
        {
            return BT::NodeStatus::FAILURE;
        } 
        else {
            geometry_msgs::PoseStamped reduce_goal;
            geometry_msgs::Quaternion orientation;
            orientation = blackboard_ptr->my_Pose.pose.orientation;

            // //* 根据是否为绕后状态，决定车orientation
            // if (blackboard_ptr->search_back_status) {
            //     orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(orientation) + M_PI + blackboard_ptr->getGimbalYawAngle());
            //     reduce_goal.pose.orientation = orientation;
            // } 
            // else {
                reduce_goal.pose.orientation = blackboard_ptr->my_Pose.pose.orientation;
            // }

            reduce_goal.header.frame_id = "map";
            reduce_goal.header.stamp = ros::Time::now();

            // //* 绕后状态选择背后目标点
            // if (blackboard_ptr->search_back_status){
            //     reduce_goal.pose.position.x = enemy_armor_pose.pose.position.x + 1.2 * cos(yaw);
            //     reduce_goal.pose.position.y = enemy_armor_pose.pose.position.y + 1.2 * sin(yaw);
            // } 
            // else {
                reduce_goal.pose.position.x = enemy_armor_pose.pose.position.x - 1.2 * cos(yaw);
                reduce_goal.pose.position.y = enemy_armor_pose.pose.position.y - 1.2 * sin(yaw);
            // }
            
            double enemy_x = reduce_goal.pose.position.x;
            double enemy_y = reduce_goal.pose.position.y;
            reduce_goal.pose.position.z = 0;
            unsigned int goal_cell_x, goal_cell_y;

            // if necessary add mutex lock
            // blackboard_->GetCostMap2D()->GetMutex()->lock();
            auto get_enemy_cell = blackboard_ptr->GetCostMap2D()->World2Map(enemy_x,
                                                                            enemy_y,
                                                                            goal_cell_x,
                                                                            goal_cell_y);
            // blackboard_->GetCostMap2D()->GetMutex()->unlock();

            if (!get_enemy_cell)
            {
                std::cout << "compute fight move goal Failed CostMap !" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
            auto robot_x = blackboard_ptr->my_Pose.pose.position.x;
            auto robot_y = blackboard_ptr->my_Pose.pose.position.y;
            unsigned int robot_cell_x, robot_cell_y;
            double goal_x, goal_y;
            blackboard_ptr->GetCostMap2D()->World2Map(robot_x,
                                                      robot_y,
                                                      robot_cell_x,
                                                      robot_cell_y);
            if (blackboard_ptr->GetCostMap2D()->GetCost(goal_cell_x, goal_cell_y) >= 253)
            {

                bool find_goal = false;
                //* 线性迭代 求取目标
                for (FastLineIterator line(goal_cell_x, goal_cell_y, robot_cell_x, robot_cell_x); line.IsValid(); line.Advance())
                {
                    auto point_cost = blackboard_ptr->GetCostMap2D()->GetCost((unsigned int)(line.GetX()), (unsigned int)(line.GetY())); // current point's cost

                    if (point_cost >= 253)
                    {
                        continue;
                    }
                    else
                    {
                        find_goal = true;
                        blackboard_ptr->GetCostMap2D()->Map2World((unsigned int)(line.GetX()),
                                                                  (unsigned int)(line.GetY()), goal_x, goal_y);

                        reduce_goal.pose.position.x = goal_x;
                        reduce_goal.pose.position.y = goal_y;
                        break;
                    }
                }
                if (find_goal)
                {
                    blackboard_ptr->my_goal = reduce_goal;
                    std::cout << "Get ***************** goal" << std::endl;
                }
                else
                {
                    std::cout << "compute fight move goal" << std::endl;
                    return BT::NodeStatus::FAILURE;
                }
            } else {
                blackboard_ptr->my_goal = reduce_goal;
                std::cout << "Get ***************** goal" << std::endl;
            }
        }
  }
  catch (tf::LookupException &ex) {
    ROS_ERROR("No Transform Error looking up robot pose: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }
  catch (tf::ConnectivityException &ex) {
    ROS_ERROR("Connectivity Error looking up robot pose: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }
  catch (tf::ExtrapolationException &ex) {
    ROS_ERROR("Extrapolation Error looking up robot pose: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
    
    // //* 距离不适合射击，计算一个移动目标进行射击
    // /** 直接选择目标地人作为目的地，进行跟随 */
    // blackboard_ptr->my_goal = blackboard_ptr->enemy_suitable_shoot->enemy_pose;
    // blackboard_ptr->my_goal.header.frame_id = "map";
    // return BT::NodeStatus::SUCCESS;
}

/**
 * ? 行为节点 ActionNode
 * ? 计算巡逻的移动目的地
 * ! 选取最佳击打位置
 */
BT::NodeStatus Compute_Patrol_MoveGoal() {
    //todo 计算巡逻线的起点[ 移动到巡逻线的起点 ]
    //******
    return BT::NodeStatus::SUCCESS;
}

/**
 * ? 行为节点 ActionNode
 * ? 计算前往加成区的移动目的地
 * ! 选取最佳击打位置
 */
BT::NodeStatus Compute_Addition_MoveGoal() {
    //todo 计算该机器人应该前往的加成区域
    //*******
    //* 从黑板获取加成区位置
    if (!blackboard_ptr->if_addition_exist) //* 加成区存在
    {
        blackboard_ptr->my_addition_goal_index = 6;
        return BT::NodeStatus::FAILURE;
    }
    if (!blackboard_ptr->if_Bullets_Supply_Active) {   //* 血量加成区未激活
        if (!blackboard_ptr->if_know_friend_pose) {
            if (blackboard_ptr->my_addition_goal_index != blackboard_ptr->Bullets_Supply - 1)
            {
                blackboard_ptr->my_addition_goal_index = blackboard_ptr->Bullets_Supply - 1;
            } 
            blackboard_ptr->my_goal
                = blackboard_ptr->F_1_6[blackboard_ptr->my_addition_goal_index]; //* 移动目标
            return BT::NodeStatus::SUCCESS;
        }
        double distance_me_2_hp_recovery
            = Compute_Distance_BetweenBoth(
                blackboard_ptr->my_Pose.pose.position.x,
                blackboard_ptr->my_Pose.pose.position.y,
                blackboard_ptr->F_1_6[blackboard_ptr->Bullets_Supply - 1 ].pose.position.x,
                blackboard_ptr->F_1_6[blackboard_ptr->Bullets_Supply - 1 ].pose.position.y
            );   
        double distance_friend_2_hp_recovery
            = Compute_Distance_BetweenBoth(
                blackboard_ptr->friend_Pose.pose.position.x,
                blackboard_ptr->friend_Pose.pose.position.y,
                blackboard_ptr->F_1_6[blackboard_ptr->Bullets_Supply -1 ].pose.position.x,
                blackboard_ptr->F_1_6[blackboard_ptr->Bullets_Supply - 1 ].pose.position.y
            );
        
        if (distance_me_2_hp_recovery <= distance_friend_2_hp_recovery) {
            //* 加成去编号以及目标位姿
            if (blackboard_ptr->my_addition_goal_index != blackboard_ptr->Bullets_Supply - 1)
            {
                blackboard_ptr->my_addition_goal_index = blackboard_ptr->Bullets_Supply - 1;
                blackboard_ptr->my_goal
                    = blackboard_ptr->F_1_6[blackboard_ptr->my_addition_goal_index]; //* 移动目标
            }
        }
        else {
            if (!blackboard_ptr->if_HP_Recovery_Active) {
                //* 加成区编号以及目标位姿
                if (blackboard_ptr->my_addition_goal_index != blackboard_ptr->HP_Recovery - 1) {
                    blackboard_ptr->my_addition_goal_index = blackboard_ptr->HP_Recovery - 1;
                    
                    blackboard_ptr->my_goal
                        = blackboard_ptr->F_1_6[blackboard_ptr->my_addition_goal_index]; //* 移动目标
                }       
            }
            else {
                //* 哪也不去 等待下次计算更新 或者 局势更新
                blackboard_ptr->my_addition_goal_index = 6;
                return BT::NodeStatus::FAILURE;
            }
        }
    }
    else {  //* 子弹加成区未激活
        if (!blackboard_ptr->if_know_friend_pose) {
            if (blackboard_ptr->my_addition_goal_index != blackboard_ptr->HP_Recovery - 1)
            {
                blackboard_ptr->my_addition_goal_index = blackboard_ptr->HP_Recovery - 1;
            } 
            blackboard_ptr->my_goal
                = blackboard_ptr->F_1_6[blackboard_ptr->my_addition_goal_index]; //* 移动目标
            return BT::NodeStatus::SUCCESS;
        }
        double distance_me_2_bullets_supply
            = Compute_Distance_BetweenBoth(
                blackboard_ptr->my_Pose.pose.position.x,
                blackboard_ptr->my_Pose.pose.position.y,
                blackboard_ptr->F_1_6[blackboard_ptr->HP_Recovery - 1].pose.position.x,
                blackboard_ptr->F_1_6[blackboard_ptr->HP_Recovery - 1].pose.position.y
            );
        double distance_friend_2_bullets_supply
            = Compute_Distance_BetweenBoth(
                blackboard_ptr->friend_Pose.pose.position.x,
                blackboard_ptr->friend_Pose.pose.position.y,
                blackboard_ptr->F_1_6[blackboard_ptr->HP_Recovery - 1].pose.position.x,
                blackboard_ptr->F_1_6[blackboard_ptr->HP_Recovery - 1].pose.position.y
            );

        if (distance_me_2_bullets_supply <= distance_friend_2_bullets_supply) {
            if (blackboard_ptr->my_addition_goal_index != blackboard_ptr->HP_Recovery - 1) {
                //* 加成区编号以及目标位姿
                blackboard_ptr->my_addition_goal_index = blackboard_ptr->HP_Recovery - 1;
                blackboard_ptr->my_goal
                    = blackboard_ptr->F_1_6[blackboard_ptr->my_addition_goal_index]; //* 移动目标
            }
        }
        else {
            //* 哪也不去 等待下次计算更新 或者 局势更新
            blackboard_ptr->my_addition_goal_index = 6;
            return BT::NodeStatus::FAILURE;
        }
    }
    return BT::NodeStatus::SUCCESS;
}

/**
 * ? 行为节点 ActionNode
 * ? 计算逃跑的移动目的地
 * ! 选取最佳击打位置
 */
BT::NodeStatus Compute_Escape_MoveGoal() {
    //todo 计算躲避敌人攻击的移动目的地
    //*******
    return BT::NodeStatus::SUCCESS;
}


/**
 * ? 计算撤出加成/惩罚区的目标位置
 *
 */
BT::NodeStatus Compute_Withdraw_Addition_MoveGoal() {
    //todo buff即将刷新 计算撤出加成/惩罚区域的移动目的地
    bool if_red = (blackboard_ptr->ID < 100) ? true : false;
    blackboard_ptr->my_goal = blackboard_ptr->my_boot_pose;
    return BT::NodeStatus::SUCCESS;
    switch (blackboard_ptr->relative_addition_area_number) {
    case 1:
        if (if_red) {
            
        }
        break;
    case 2:
        break;
    case 3:
        if (if_red) {
            
        }
        break;
    case 4:
        break;
    case 5:
        break;
    case 6:
        break;
    default:
        break;
    }
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief Update Addition Active State
 * 
 * @return BT::NodeStatus 
 */
BT::NodeStatus UpdateAdditionActiveState() {
    if (blackboard_ptr->my_addition_goal_index == blackboard_ptr->HP_Recovery - 1)
        blackboard_ptr->if_HP_Recovery_Active = true;
    else {
        blackboard_ptr->if_Bullets_Supply_Active = true;
    }
    return BT::NodeStatus::SUCCESS;
}


/**
 * @brief Set the Goal object
 * * 设置移动目标
 * @return BT::NodeStatus 
 */
BT::NodeStatus SetGoal(){
    if (blackboard_ptr->game_status == GameStatus::End) {
        blackboard_ptr->my_goal = blackboard_ptr->my_boot_pose;
        if (blackboard_ptr->ID == 2 || blackboard_ptr->ID == 102)
        {
            blackboard_ptr->my_goal = blackboard_ptr->my_boot_pose;
            blackboard_ptr->my_goal.pose.position.y = 4.48 - blackboard_ptr->my_goal.pose.position.y;
        }
    }
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief Set the Search Back Status object
 * * 设置绕后状态
 * @return BT::NodeStatus 
 */
BT::NodeStatus SetSearchBackStatus()
{
    blackboard_ptr->search_back_status = true;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CancelSearchBackStatus() 
{
    blackboard_ptr->search_back_status = false;
    return BT::NodeStatus::SUCCESS;
}


/**
 * @brief get bullets addition goal
 * ? 获取子弹加成区目标位姿
 */
BT::NodeStatus GetBulletsAdditionGoal(/* BT::TreeNode &self */)
{
    // self.setOutput<uint8_t>("bullets_addition_goal", blackboard_ptr->Bullets_Supply);
    if (blackboard_ptr->Bullets_Supply < 1 || blackboard_ptr->if_Bullets_Supply_Active) {
        return BT::NodeStatus::FAILURE;
    }
    //* 设置加成区
    blackboard_ptr->my_addition_goal_index = (blackboard_ptr->ID < 100) ? (blackboard_ptr->Bullets_Supply - 1) : (6 - blackboard_ptr->Bullets_Supply);
    blackboard_ptr->my_goal = blackboard_ptr->F_1_6[blackboard_ptr->my_addition_goal_index];
    std::cout << "目的---子弹加成区编号：" << + blackboard_ptr->Bullets_Supply << std::endl;
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief get HP addition goal
 * ? 获取血量加成区目标位姿
 */
BT::NodeStatus GetHPAdditionGoal(/* BT::TreeNode &self */ ){
    if (blackboard_ptr->HP_Recovery < 1 || blackboard_ptr->if_HP_Recovery_Active){
        return BT::NodeStatus::FAILURE;
    }
    //* 设置加成区
    blackboard_ptr->my_addition_goal_index = (blackboard_ptr->ID < 100) ? (blackboard_ptr->HP_Recovery - 1) : (6 - blackboard_ptr->HP_Recovery);
    blackboard_ptr->my_goal = blackboard_ptr->F_1_6[blackboard_ptr->my_addition_goal_index];
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief Set the Addition Adjust Goal object
 * 
 * @return BT::NodeStatus 
 */
BT::NodeStatus SetAdditionAdjustGoal() {
    switch (blackboard_ptr->my_addition_goal_index)
    {
    case 0:
        blackboard_ptr->my_goal.pose.position.x = blackboard_ptr->my_goal.pose.position.x + 0.15;
        blackboard_ptr->my_goal.pose.position.y = blackboard_ptr->my_goal.pose.position.y - 0.1;
        break;
    case 1:
        blackboard_ptr->my_goal.pose.position.x = blackboard_ptr->my_goal.pose.position.x + 0.05;
        blackboard_ptr->my_goal.pose.position.y = blackboard_ptr->my_goal.pose.position.y - 0.1;
        break;
    case 2:
        blackboard_ptr->my_goal.pose.position.x = blackboard_ptr->my_goal.pose.position.x + 0.05;
        blackboard_ptr->my_goal.pose.position.y = blackboard_ptr->my_goal.pose.position.y + 0.1;
        break;
    case 3:
        blackboard_ptr->my_goal.pose.position.x = blackboard_ptr->my_goal.pose.position.x - 0.05;
        blackboard_ptr->my_goal.pose.position.y = blackboard_ptr->my_goal.pose.position.y - 0.1;
        break;
    case 4:
        blackboard_ptr->my_goal.pose.position.x = blackboard_ptr->my_goal.pose.position.x - 0.05;
        blackboard_ptr->my_goal.pose.position.y = blackboard_ptr->my_goal.pose.position.y + 0.1;
        break;
    case 5:
        blackboard_ptr->my_goal.pose.position.x = blackboard_ptr->my_goal.pose.position.x - 0.15;
        blackboard_ptr->my_goal.pose.position.y = blackboard_ptr->my_goal.pose.position.y + 0.1;
        break;
    default:
        std::cout << "加成区编号错误" << std::endl;
        return BT::NodeStatus::FAILURE;
        break;
    }
    return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus SetGameEnd(){
    blackboard_ptr->game_end_ = true;
    return BT::NodeStatus::SUCCESS;
}


/**
 * ? 功能函数
 * ? 计算一次设计多少发子弹
 * ! 得到数字 uint8_t
 */
BT::NodeStatus Compute_ShootNumber(BT::TreeNode &self) {
    // double distance = 
    //     Compute_Distance_Point(blackboard_ptr->enemy_suitable_shoot->enemy_camera_pose.x,
    //         blackboard_ptr->enemy_suitable_shoot->enemy_camera_pose.y,
    //         blackboard_ptr->enemy_suitable_shoot->enemy_camera_pose.z);
    self.setOutput<uint8_t>("shootnumber", blackboard_ptr->shoot_number);
    return BT::NodeStatus::SUCCESS;
}


/**
 * ? 功能函数
 * ? 计算射击到敌人的概率
 * ! 得到概率 做判断
 */
double Compute_My_HittingAccuracy_(geometry_msgs::PoseStamped my_pose,
    geometry_msgs::PoseStamped enemy_pose,
    bool if_enemy_armor_detected)
{
    if (!if_enemy_armor_detected)
        return 0;
    //* 计算距离
    double probability = 0;
    //* 计算敌人与自身的距离
    double distance = 0;
    distance = sqrt(pow((my_pose.pose.position.x - enemy_pose.pose.position.x), 2) +
        pow((my_pose.pose.position.y - enemy_pose.pose.position.y), 2));
    if (distance > 2)
        probability = 1 - 0.125 * (distance - 2);
    else
        probability = 100 - 0.125 * (2 - distance);
    return probability;
}

/**
 * ? 功能函数
 * ? 计算被敌人射击的概率
 * ! 得到概率 做判断
 */
double Compute_Enemy_HittingAccuracy_(geometry_msgs::PoseStamped my_pose,
    geometry_msgs::PoseStamped enemy_pose)
{
    double probability = 0;
    //* 计算敌人与自身的距离
    double distance = 0;
    distance = sqrt(pow((my_pose.pose.position.x - enemy_pose.pose.position.x), 2) +
        pow((my_pose.pose.position.y - enemy_pose.pose.position.y), 2));
    if (distance > 2)
        probability = 100 - 12.5 * (distance - 2);
    else
        probability = 100 - 12.5 * (2 - distance);
    return probability;
}



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
