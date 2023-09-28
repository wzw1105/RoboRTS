
#ifndef FIND_AVAILABLE_SHOOT_POSITION_WITH_GUARD_MSG_H
#define FIND_AVAILABLE_SHOOT_POSITION_WITH_GUARD_MSG_H

#include <ros/ros.h>
#include <cmath>
#include <cstdlib>
#include <ctime> 
#include "geometry_msgs/PoseStamped.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "../blackboard/blackboard.h"
#include "roborts_msgs/CarMsgs.h"
#include "roborts_msgs/CarMsg.h"

namespace roborts_decision{

float rec_x[6] = {1.5, 2.75, 4, 5.25, 6.5, 8.1};
float rec_y[4] = {1, 2.2, 3.4, 4.5};

int optimal_shoot_position[6][4][4][2] = {
    // 0 - 2 and 1 - 3 respectively for two robots
    {{{0, 1}, {1, 1}, {-1, -1}, {-1, -1}},  {{1, 1}, {0, 2}, {1, 2}, {-1, -1}},  {{0, 1}, {1, 2}, {1, 1}, {1, 3}},   {{1, 3}, {1, 2}, {-1, -1}, {-1, -1}}}, //column 0
    {{{2, 0}, {0, 1}, {2, 1}, {1, 1}},      {{1, 0}, {1, 2}, {2, 1}, {0, 1}},    {{2, 2}, {1, 1}, {1, 3}, {0, 2}},   {{1, 2}, {2, 2}, {0, 3}, {2, 3}}}, //column1
    {{{1, 0}, {3, 0}, {1, 1}, {2, 1}},      {{1, 1}, {3, 1}, {2, 0}, {2, 2}},    {{2, 1}, {2, 3}, {3, 2}, {1, 2}},   {{1, 2}, {3, 3}, {1, 3}, {2, 2}}}, //2
    {{{4, 0}, {2, 0}, {4, 1}, {3, 1}},      {{3, 2}, {4, 1}, {2, 1}, {3, 0}},    {{3, 1}, {2, 2}, {4, 2}, {3, 3}},   {{4, 2}, {3, 2}, {4, 3}, {2, 3}}}, //3
    {{{3, 0}, {4, 1}, {3, 1}, {5, 0}},      {{4, 0}, {4, 2}, {3, 1}, {3, 2}},    {{4, 1}, {3, 2}, {3, 1}, {4, 3}},   {{3, 3}, {4, 2}, {3, 2}, {5, 2}}}, //4
    {{{4, 0}, {4, 1}, {-1, -1}, {-1, -1}},  {{4, 0}, {5, 2}, {4, 1}, {4, 2}},    {{4, 1}, {5, 3}, {4, 2}, {-1 ,-1}}, {{5, 2}, {4, 2}, {-1, -1}, {-1, -1}}} //5
};

int optimal_shoot_position_num[6][4] = {
    {2, 3, 4, 2},
    {4, 4, 4, 4},
    {4, 4, 4, 4},
    {4, 4, 4, 4},
    {4, 4, 4, 4},
    {2, 4, 3, 2},
};


class FindAvailableShootPositionWithGuardMsg : public BT::SyncActionNode{
public:
	FindAvailableShootPositionWithGuardMsg(const std::string& name) : BT::SyncActionNode(name,{}) {}

    BT::NodeStatus tick() override{
        //choose an enemy from list, Lurk status considered
        roborts_msgs::CarMsg result;
        if( !ChooseOptimalGoal(blackboard_ptr_ -> guard_msgs, result) ) {
            ROS_INFO("No valid enemy in guard view!!!");
            return BT::NodeStatus::FAILURE;
        }
        int x_index, y_index;
        ROS_INFO("Choose enemy in position: x = %.1lf x = %.1lf", result.x, result.y);

        //get index in map
        if(!GetAreaIndex(result.x, result.y, x_index, y_index)) {
            ROS_INFO("Fail to get enemy area index with guard msg: x = %.1lf y = %.1lf !", result.x, result.y);
            return BT::NodeStatus::FAILURE;
        }
        if(x_index < 0 || x_index > 5 || y_index < 0 || y_index > 3) return BT::NodeStatus::FAILURE;

        //get shoot position index in optimal_shoot_position array
        int choosed_index = ChooseShootPosition(x_index, y_index);
        
        //calculate pose for shoot position
        if(!CalculateAreaCenter(optimal_shoot_position[x_index][y_index][choosed_index][0], optimal_shoot_position[x_index][y_index][choosed_index][1], blackboard_ptr_ -> enemy_area_goal)){
            ROS_INFO("Can't calculate enemy center!");
            return BT::NodeStatus::FAILURE;
        }
        double yaw_ = atan2(result.y - blackboard_ptr_ -> enemy_area_goal.pose.position.y, result.x - blackboard_ptr_ -> enemy_area_goal.pose.position.x);
        ROS_INFO("Choose position : x = %.1lf, y = %.1lf, yaw = %.1lf for shoot.", blackboard_ptr_ -> enemy_area_goal.pose.position.x, blackboard_ptr_ -> enemy_area_goal.pose.position.y, yaw_);
        blackboard_ptr_ -> enemy_area_goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_);

        return BT::NodeStatus::SUCCESS;
    }

    void init(std::shared_ptr<BlackBoard> blackboard) {
        this -> blackboard_ptr_ = blackboard;
    }

private:
    std::shared_ptr<BlackBoard> blackboard_ptr_;

    //functions
    bool GetAreaIndex(float x, float y, int &x_index, int &y_index) { //get integer index of pos(x , y), stored in (x_index, y_index)
        x_index = -1, y_index = -1;
        if(x < 0 || y < 0) {
            ROS_INFO("x = %.1f y = %.1f : negative position find!", x, y);
            return false;
        }
        for(int i = 0; i < 6; i++) if(x <= rec_x[i]) x_index = i;
        for(int i = 0; i < 4; i++) if(y <= rec_y[i]) y_index = i;
        if(x_index == -1 || y_index == -1) return false;
        return true;
    }

    bool CalculateAreaCenter(int x_index, int y_index, geometry_msgs::PoseStamped &shoot_pose) {
        if(x_index < 0 || y_index < 0 || x_index > 5 || y_index > 3) return false;

        //x of center
        if(x_index == 0) shoot_pose.pose.position.x = rec_x[0] / 2;
        else shoot_pose.pose.position.x = rec_x[x_index - 1] + (rec_x[x_index] - rec_x[x_index - 1]) / 2;

        //y of center
        if(y_index == 0) shoot_pose.pose.position.y = rec_y[0] / 2;
        else shoot_pose.pose.position.y = rec_y[y_index - 1] + (rec_y[y_index] - rec_y[y_index - 1]) / 2;
        
        return true;
    }

    //
    bool ChooseOptimalGoal(roborts_msgs::CarMsgs &all_cars, roborts_msgs::CarMsg &choosed_goal) {
        int num_enemy_detected = 0;
        roborts_msgs::CarMsg enermy[2];
        //Normal Status
        if(blackboard_ptr_ -> lurk_status < 2) {
            for(auto &car : all_cars.car_msgs) {
                if(car.id == 0) continue;
                if(num_enemy_detected == 2) break;
                //0- red 1-blue
                if(!same_color_with_me(car.color)) {
                    enermy[num_enemy_detected++ ] = car;
                } 
            }
            if(num_enemy_detected == 0) return false;
            else if(num_enemy_detected == 1) {
                choosed_goal = enermy[0];
                return true;
            }else{
                if(EvaluateCarScore(enermy[0]) > EvaluateCarScore(enermy[1])) choosed_goal = enermy[0];
                else choosed_goal = enermy[1];
                return true;
            }
        }else {
            for(auto &car : all_cars.car_msgs) {
                if(car.id == 0) continue;
                if(num_enemy_detected == 2) break;
                //0- red 1-blue
                if(same_color_with_me(car.color) || car.id == blackboard_ptr_ -> self_id || blackboard_ptr_ -> friend_dead) { // 1 vs 1 and 2 vs 2 or vs both if friend dead 
                    enermy[num_enemy_detected++ ] = car;
                } 
            }

            if(num_enemy_detected == 0) return false;
            else if(num_enemy_detected == 1) {
                choosed_goal = enermy[0];
                return true;
            }else{
                if(EvaluateCarScore(enermy[0]) > EvaluateCarScore(enermy[1])) choosed_goal = enermy[0];
                else choosed_goal = enermy[1];
                return true;
            }
        }
    }

    bool same_color_with_me(char c) {
        return (blackboard_ptr_ -> self_color == 0 && c == 'r' ) || (blackboard_ptr_ -> self_color == 1 && c == 'b');
    }

    float EvaluateCarScore(roborts_msgs::CarMsg &car) {
        float self_x = blackboard_ptr_ -> self_pose.pose.position.x, self_y = blackboard_ptr_ -> self_pose.pose.position.y;
        float enemy_x = car.x, enermy_y = car.y; 
        float dis_score = 2 * (10.0 - sqrt((self_x - enemy_x) * (self_x - enemy_x) + (self_y - enermy_y) * (self_y - enermy_y)) )/ 10.0;  //with amx: 2 score
        float hp_score = (2000.0 - blackboard_ptr_ -> robot_hp[(blackboard_ptr_ -> self_color + 1) % 2][car.id]) / 2000.0; //with max : 1 score
        float bullets_score = (700 -  blackboard_ptr_ -> robot_bullets[(blackboard_ptr_ -> self_color + 1) % 2][car.id]) / 1400.0; //with max : 0.5 score
        return dis_score + hp_score + bullets_score;
    }

    int ChooseShootPosition(const int &x_index, const int &y_index){
        if(blackboard_ptr_ -> friend_dead) return rand() % optimal_shoot_position_num[x_index][y_index];
        else{
            if(optimal_shoot_position_num[x_index][y_index] == 2) { //car1: 0, car2: 1
                return blackboard_ptr_ -> self_id;
            }else if(optimal_shoot_position_num[x_index][y_index] == 3) {  //car1: 0 2, car2: 1
                if(blackboard_ptr_ -> self_id == 0) {
                    return rand() % 2 ? 2 : 0;
                }else{
                    return 1;
                }
            }else{ //car1 : 0, 2, car2 : 1, 3 
                return blackboard_ptr_ -> self_id + (rand() % 2 ? 2 : 0);
            }
        }
    }
};

}

#endif