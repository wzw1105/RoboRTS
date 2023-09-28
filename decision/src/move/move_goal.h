#ifndef _MOVE_GOAL_H_
#define _MOVE_GOAL_H_

#include "share_head.h"
#include "enum_class.h"

using namespace BT;
using namespace RTS_DECISION;

class MoveGoal : public CoroActionNode {
  public:
    MoveGoal(const std::string& name):
        CoroActionNode(name,{})
    {
        //* 退出标志
        _halt_requested.store(false);
    }
    //* 析构函数
    ~MoveGoal();

    NodeStatus tick() override;
    void halt() override;

    void init(ChassisExecutor* &chassis_executor, GimbalExecutor *gimbal_executor);

  private:
    std::atomic_bool _halt_requested;
    ChassisExecutor* chassis_executor_;
    GimbalExecutor *gimbal_executor_;
    geometry_msgs::PoseStamped goal_position_;
};


#endif