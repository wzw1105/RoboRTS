/*
 * @content: 保持安静
 * @Description: file 保持安静
 * @author: Chen
 */
#include "./awaken.h"

BT::NodeStatus Awaken::tick() {
    if (awake)
        return BT::NodeStatus::SUCCESS;

    roborts_msgs::FricWhl fricwhl_srv_msg;
    fricwhl_srv_msg.response.received = false;
    ros::service::waitForService("cmd_fric_wheel");
    ros::service::waitForService("cmd_shoot");
    fricwhl_srv_msg.request.open = true;
    while (!fricwhl_srv_msg.response.received)
    {
        this->FricWhl_service_client_.call(fricwhl_srv_msg);
    }

	roborts_msgs::ShootCmd shoot_cmd;
	shoot_cmd.request.mode = 0;
	shoot_cmd.request.number = 0;

    gimbal_executor_->Execute(shoot_cmd);

    if (fricwhl_srv_msg.response.received) {
        this->awake = true;
        return BT::NodeStatus::SUCCESS;
    }
    else {
        return BT::NodeStatus::FAILURE;
    }

}

Awaken::~Awaken(){}


void Awaken::init(GimbalExecutor *&gimbal_executor){
   //* 云台控制器
   this->gimbal_executor_ = gimbal_executor;
}
