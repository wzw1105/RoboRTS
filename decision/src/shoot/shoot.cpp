/*
 * @content: 保持安静
 * @Description: file 保持安静
 * @author: Chen
 */
/*
 * @content: 保持安静
 * @Description: file 保持安静
 * @author: Chen
 */
#include "./shoot.h"

NodeStatus Shoot::tick() {
    std::cout<<"shoot start!" <<std::endl;
    ros::Rate loop_rate(4);
    this->_halt_request.store(false);
    roborts_msgs::GimbalAngle gimbal_angle;
    gimbal_angle.yaw_mode = true;
    gimbal_angle.pitch_mode = false;
    double last_yaw_angle = 0.0;

    while (!_halt_request && ros::ok())
    {
        auto shoot_number_msg = getInput<uint8_t>("number");
        if (!shoot_number_msg)
        {
            throw BT::RuntimeError("missing required input [message]: ", shoot_number_msg.error());
        }
        else
        {
            shoot_cmd_msg.request.mode = 1;
            shoot_cmd_msg.request.number = shoot_number_msg.value();
            shoot_cmd_msg.response.received = false;
            
            gimbal_executor_->Execute(shoot_cmd_msg);
        }

        loop_rate.sleep();

        //* 挂起线程，tick()返回running
        CoroActionNode::setStatusRunningAndYield();
    }

    this->stop();
    std::cout << "shoot end" << std::endl;
    return _halt_request ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
}

BT::PortsList Shoot::providedPorts(){
    return {BT::InputPort<uint8_t>("number")};
}

void Shoot::halt() {
    this->stop();
    CoroActionNode::halt();
}

void Shoot::init(GimbalExecutor*& gimbal_executor) {
    this->gimbal_executor_ = gimbal_executor;
}

void Shoot::stop(){
    this->_halt_request.store(true);
}

//* 射击调整
void Shoot::Shoot_Control_CallBack_(const roborts_msgs::ShootControl::ConstPtr &shoot_control){
    if (!this->_halt_request)
    {
        roborts_msgs::GimbalAngle gimbal_angle;
        gimbal_angle.pitch_mode = false;
        gimbal_angle.yaw_mode = true;
        gimbal_angle.yaw_angle = shoot_control->yaw_angle;
        gimbal_angle.pitch_angle = shoot_control->pitch_angle;
        this->gimbal_executor_->Execute(gimbal_angle);
        gimbal_angle.yaw_angle = 0.5 * gimbal_angle.yaw_angle;
        this->gimbal_executor_->Execute(gimbal_angle);
    }
}