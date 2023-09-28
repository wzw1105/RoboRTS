/*
 * @content: 保持安静
 * @Description: file 保持安静is-
 * @author: Chen
 */
/*
 * @content: 保持安静
 * @Description: file 保持安静
 */
#include "./turn.h"

/**
 * @brief 
 * uint8 FORWARD = 0
 * uint8 LEFT = 1
 * uint8 BACKWARD = 2
 * uint8 RIGHT = 3
 * @return BT::NodeStatus 
 */
BT::NodeStatus Turn::tick() {
    std::cout << "turning!" << std::endl;
    ros::Rate loop_rate(50);
    double flucatuation = 0.1545329;
    double chassis_angle_current = 0;
    double goal_chassis_angle = 0;
    double chassis_yaw_angle = 0;
    this->chassis_executor_->Cancel();
    
    ENUM_CLASS::BehaviorState executor_state;
    //* 旋转角度
    this->chassis_msgs_.angular.z = M_PI;
    //* 开始旋转前 底盘的角度 （从里程计获取）
    chassis_yaw_angle =  tf::getYaw(blackboard_ptr->odom_info_.pose.pose.orientation);
    //* 根据受攻击的装甲板 
    switch (blackboard_ptr->robot_damage.damage_source)
    {
    case 0:
        return BT::NodeStatus::SUCCESS;
        break;
    case 1:
        //* 左转90度
        
        goal_chassis_angle = (chassis_yaw_angle > M_PI_2) ? (chassis_yaw_angle + M_PI_2 - M_PI * 2) : (chassis_yaw_angle + M_PI_2);
        break;
    case 2:
        //* 旋转180度
        
        goal_chassis_angle = (chassis_yaw_angle > 0) ? (chassis_yaw_angle - M_PI) : (chassis_yaw_angle + M_PI);
        break;
    case 3:
        //* 右转90度
        
        this->chassis_msgs_.angular.z = -this->chassis_msgs_.angular.z;
        goal_chassis_angle = (chassis_yaw_angle < -M_PI_2) ? (chassis_yaw_angle - M_PI_2 + M_PI * 2) : (chassis_yaw_angle - M_PI_2);
        break;
    default:
        break;
    }
    //* 旋转车身 然后云台的yaw轴归零
    while(ros::ok()){
        executor_state = this->chassis_executor_->Update();
        if (executor_state == BehaviorState::FAILURE || executor_state == BehaviorState::SUCCESS)
            break;
        
        //* 获取yaw轴角度
        chassis_angle_current = tf::getYaw(blackboard_ptr->odom_info_.pose.pose.orientation);
        //* 判断停止条件
        if ((chassis_angle_current >= goal_chassis_angle - flucatuation) 
            && (chassis_angle_current <= goal_chassis_angle + flucatuation)
            ){
            this->chassis_executor_->Cancel();
            break;
        }
        //* 执行旋转
        this->twist_accl.twist = this->chassis_msgs_;
        // this->chassis_executor_->Execute(this->chassis_msgs_);
        this->chassis_executor_->Execute(this->twist_accl);

        // //* 频率控制
        loop_rate.sleep();
    }
    
    //* 发布云台调整消息--云台归零 yaw_mode false
    this->gimbal_executor_->Cancel();
    this->chassis_executor_->Cancel();

    return executor_state == BehaviorState::SUCCESS ?  BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void Turn::init(ChassisExecutor* &chassis_executor,GimbalExecutor* &gimbal_executor){
    this->chassis_executor_ = chassis_executor;
    this->gimbal_executor_ = gimbal_executor;
}

Turn::~Turn(){
    this->chassis_executor_->Cancel();
    this->gimbal_executor_->Cancel();
}