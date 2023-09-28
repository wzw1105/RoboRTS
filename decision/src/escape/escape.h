/*
 * @Author: chen
 * @Date: 2022-04-01 17:47:20
 * @LastEditTime: 2022-05-06 10:01:01
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置
 * @FilePath: \roborts_ws\src\test_pkg\src\skills\escape.h
 */
#include "share_head.h"
using namespace BT;
using namespace RTS_DECISION;

class Escape : public CoroActionNode {
	
private:
	std::atomic_bool _halt_requested;
	ChassisExecutor* chassis_executor_;
	GimbalExecutor *gimbal_executor_;
	geometry_msgs::PoseStamped goal_position_;
	
public:
	Escape(const std::string& name):
        CoroActionNode(name,{})
    {
        //* 退出标志
        _halt_requested.store(false);
	};

	//* 循环程序
	NodeStatus tick() override;
	//* 中止函数
	void halt() override;
	//* 初始化函数
    void init(ChassisExecutor* &chassis_executor, GimbalExecutor* &gimbal_executor);
};
