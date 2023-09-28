#include "simple_actions.h"

SimpleActions::SimpleActions(){
    Frichwl_service_client_ = nh_.serviceClient<roborts_msgs::FricWhl>("cmd_fric_wheel");
}

/**
 * @brief 唤醒摩擦轮
 * 
 * @return BT::NodeStatus 
 */
BT::NodeStatus SimpleActions::AwakeFrichwl(){
    roborts_msgs::FricWhl frichwl_srv_msg;
    frichwl_srv_msg.request.open = true;
    Frichwl_service_client_.call(frichwl_srv_msg);
    if (frichwl_srv_msg.response.received) {
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}