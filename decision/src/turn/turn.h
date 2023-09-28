/*
 * @content: 保持安静
 * @Description: file 保持安静
 * @author: Chen
 */
/*
 * @content: 保持安静
 * @Description: file 保持安静
 */
#include "share_head.h"

using namespace BT;
using namespace RTS_DECISION;

class Turn : public SyncActionNode {
 public:
    Turn(const std::string& name)
      : BT::SyncActionNode(name, {})
    {
        this->gimbal_zero_msgs_.yaw_mode = false;
        this->gimbal_zero_msgs_.pitch_mode = true;
        this->gimbal_zero_msgs_.yaw_angle = 0;
        this->gimbal_zero_msgs_.pitch_angle = 0;
        //* 初始化
        this->twist_accl.accel.angular.z = 0;
        this->twist_accl.accel.linear.x = 0;
        this->twist_accl.accel.linear.y = 0;

    }

    ~Turn();

    BT::NodeStatus tick() override;

    void init(ChassisExecutor* &chassis_executor,GimbalExecutor* &gimbal_executor);

private:
    geometry_msgs::Twist chassis_msgs_;
    //* 速度 
    roborts_msgs::TwistAccel twist_accl;
    roborts_msgs::GimbalAngle gimbal_zero_msgs_;
    ChassisExecutor *chassis_executor_;
    GimbalExecutor *gimbal_executor_;
};