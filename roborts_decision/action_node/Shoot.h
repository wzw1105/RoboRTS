#ifndef SHOOT_H
#define SHOOT_H

#include <ros/ros.h>
#include <cmath>
#include <mutex>
#include "roborts_msgs/GimbalAngle.h"
#include "../executor/gimbal_executor.h"
#include "roborts_msgs/ShootCmd.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "../blackboard/blackboard.h"
#include "../behavior_tree/behavior_state.h"

namespace roborts_decision{

class Shoot : public BT::CoroActionNode{
public:
	Shoot(const std::string& name) : CoroActionNode(name,{}) {
        _halt_requested.store(false);
        last_shoot_time = ros::Time::now() - ros::Duration(600);
	}
    BT::NodeStatus tick() override{    
        while (!_halt_requested && ros::ok()) {
            ros::Time now_ = ros::Time::now();
            if(now_.toSec() - last_shoot_time.toSec() < 1.0 * last_shoot_num / 10.0) setStatusRunningAndYield();  //
            //float x = 0, y = 0;
            int shoot_num = 0;
            //bool this_frame_detected = false;
            shoot_num = this -> blackboard_ptr_ -> shoot_num;
            //float dis = sqrt(x * x + y * y);
            //float next_pitch = -GetPitch(dis, -h , blackboard_ptr_ -> shoot_speed);
            //float next_yaw = (float) (atan2(y, x));

            //if(this_frame_detected) PublishPitchYawMsgs(next_yaw, next_pitch);
            if(shoot_num != 0) {
                last_shoot_num = shoot_num;
                last_shoot_time = ros::Time::now(); //update last_shoot_time if shoot
                CallForShootService(shoot_num);
            }
            setStatusRunningAndYield();
        }
        // reinit the yaw and pitch
        this->gimbal_executor_->Cancel();
        //shoot stop
        shootSrv.request.mode = 0;
        shootSrv.request.number = 0;
        this -> blackboard_ptr_ -> shoot_client.call(shootSrv);

        //halt或全局规划器执行状态为失败
        return _halt_requested ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }
    

    void halt() override{
        // reinit the yaw and pitch
        this->gimbal_executor_->Cancel();
        //shoot stop
        shootSrv.request.mode = 0;
        shootSrv.request.number = 0;
        this -> blackboard_ptr_ -> shoot_client.call(shootSrv);

        this -> blackboard_ptr_ -> shoot_num = 0;


        this->_halt_requested.store(true);
        CoroActionNode::halt();
    }

    void init(std::shared_ptr<BlackBoard> blackboard, GimbalExecutor* gimbal) {
        blackboard_ptr_ = blackboard;
        gimbal_executor_ = gimbal;
        
    }

private:
    float init_k_ = 0.026;
    float GRAVITY = 9.78;
    double PI = 3.1415926535;
    float h = 0.2;

    float cur_yaw_ = 0, cur_pitch_ = 0;

    //the threshold for changing yaw and pitch
    float yaw_change_threshold = 0.01;
    float pitch_change_threshold = 0.01;
    const double yaw_change_unit_ = 0.05;

    ros::Time last_shoot_time;
    int last_shoot_num = 0;

    roborts_msgs::GimbalAngle gimbal_angle_; //publish yaw pitch
    roborts_msgs::ShootCmd shootSrv;

	std::atomic_bool _halt_requested;
    GimbalExecutor* gimbal_executor_;
    std::shared_ptr<BlackBoard>  blackboard_ptr_;

    float BulletModel(float x, float v, float angle) { //x:m,v:m/s,angle:rad
        float t, y;
        t = (float)((exp(init_k_ * x) - 1) / (init_k_ * v * cos(angle)));
        y = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
        return y;
    }

    //x:distance , y: height
    float GetPitch(float x, float y, float v) {
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
        return a;
    }

    /*
    void PublishPitchYawMsgs(float &next_yaw, float &next_pitch) {
        roborts_msgs::GimbalAngle gimbal_cmd_info;

        gimbal_cmd_info.yaw_mode = true;
        gimbal_cmd_info.pitch_mode = false;

        if(fabs(next_yaw) < yaw_change_threshold) next_yaw = 0;
        if(fabs(next_pitch - cur_pitch_) < pitch_change_threshold) next_pitch = cur_pitch_;

        gimbal_cmd_info.yaw_angle = next_yaw;
        gimbal_cmd_info.pitch_angle = next_pitch;
        std::cout << "Pitch: " << next_pitch << " Yaw: " << next_yaw << "\n";
        gimbal_executor_ -> Execute(gimbal_cmd_info);
    }*/

    void PublishPitchYawMsgs(float &next_yaw, float &next_pitch) {
        if(fabs(next_yaw) < yaw_change_threshold) next_yaw = 0;
        if(fabs(next_pitch - cur_pitch_) < pitch_change_threshold) next_pitch = cur_pitch_;
        cur_pitch_ = next_pitch;
        std::cout << "Pitch: " << next_pitch << " Yaw: " << next_yaw << "\n";
        roborts_msgs::GimbalAngle gimbal_cmd_info;

        float tmp_yaw = 0, total_yaw = 0;
        ros::Rate loop_rate(40);
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

    bool CallForShootService(const int &shoot_num) {
        shootSrv.request.mode = 1;
        shootSrv.request.number = shoot_num;
        if(!blackboard_ptr_ -> shoot_client.call(shootSrv)) ROS_WARN ("Erros occured, can't shoot!");
        else ROS_INFO("Shoot %d bullets successfully!!! ", shoot_num);
    }

};

}

#endif