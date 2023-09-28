/*
 * @Author: your name
 * @Date: 2022-03-25 16:07:16
 * @LastEditTime: 2022-05-14 09:29:24
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: \roborts_ws\src\RoboRTS\test_pkg\src\skills\patrol.h
 */
#include "share_head.h"
using namespace BT;
using namespace RTS_DECISION;

class Patrol : public CoroActionNode
{
private:
	std::atomic_bool _halt_requested;
	ChassisExecutor *chassis_executor_;
	GimbalExecutor *gimbal_executor_;
	uint8_t patrol_goal_serial;	//* 区域序号，按顺序前往目的地巡逻
	geometry_msgs::PoseStamped patrol_goal;

public:
	Patrol(const std::string& name):
        CoroActionNode(name,{})
    {
		this->patrol_goal_serial = 0;
        //* 退出标志
        _halt_requested.store(false);
	}
	//*析构函数
	~Patrol();

	//* override Function
	NodeStatus tick() override;
    void halt() override;

	//* init params
	void init(ChassisExecutor*& chassis_executor, GimbalExecutor*& gimbal_executor);

	//* stop function
	void stop();
};
