#include "./pub_shoot.h"

NodeStatus PubShoot::tick() {
    this->_halt_request.store(false);
    ros::Rate loop_rate(25);
    this->seq = blackboard_ptr->shoot_seq;
    roborts_msgs::ShootControl shoot_control_msg;
    while (!_halt_request && ros::ok())
    {
        if (this->seq != blackboard_ptr->shoot_seq) {
            this->seq = blackboard_ptr->shoot_seq;
            shoot_control_msg.pitch_angle = blackboard_ptr->gimbal_pitch_offset;
            shoot_control_msg.yaw_angle = blackboard_ptr->gimbal_yaw_offset;
            shoot_control_msg.shoot_mode = 1;
            shoot_control_msg.shoot_number = blackboard_ptr->shoot_number;
            this->shoot_pub_.publish(shoot_control_msg);
        }
        loop_rate.sleep();
        //* 挂起线程，tick()返回running
        setStatusRunningAndYield();
    }

    return _halt_request ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

void PubShoot::halt(){
    this->_halt_request.store(true);
    //* don't forget to call this function
    CoroActionNode::halt();
}