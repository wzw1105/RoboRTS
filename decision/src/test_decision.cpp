/*
 * @Author: chen
 * @Date: 2021-11-18 16:00:45
 * @LastEditTime: 2022-05-21 22:40:37
 * @LastEditors: Please set LastEditors
 * @Description: 程序入口文件
 * @FilePath: decision/src/test_decison.cpp
 */

#include "share_head.h"

#include "./functions/Functions.h"
#include "./move/move_goal.h"
#include "./patrol/patrol.h"
#include "./PreConditions.h"
#include "./shoot/shoot.h"
#include "./swing/swing_idle.h"
#include "./turn/turn.h"
#include "./awaken/awaken.h"
#include "./FindAvailableShootPositionWithGuardMsg.h"
#include "./MoveToEnermyArea.h"
#include "./ConditionNode.h"

#include <thread>

using namespace BT;
using namespace RTS_DECISION;

//* 全局变量，黑板参数，在此声明
BlackBoard::Ptr blackboard_ptr;// = std::make_shared<BlackBoard>();
const std::string bt_test_path = ros::package::getPath("decision") + "/src/bt_test_files/bt_test.xml";


/**
 * @brief 行为树实现
 * ! 行为树测试
 * ? 简单行为树
 * @param chassis_executor 
 * @param gimbal_executor 
 */
void BT_test(ChassisExecutor* chassis_executor, GimbalExecutor* gimbal_executor);


/**
 * @brief 程序入口
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv) {

	//* ros 初始化
	ros::init(argc, argv, "tester");

	//* 底盘动作执行器---所有节点均使用
	ChassisExecutor *chassis_executor = new RTS_DECISION::ChassisExecutor;
	GimbalExecutor *gimbal_executor = new RTS_DECISION::GimbalExecutor;

	//* 黑板数据定义
	blackboard_ptr = std::make_shared<BlackBoard>(chassis_executor,gimbal_executor);

	//* 为行为树创建线程
	// pthread_t tids;
	// int ret = pthread_create(&tids, NULL, BT_test, NULL);

	/**
	 * @brief 为行为树另开线程运行
	 * 
	 * @return std::thread 
	 */
	std::thread thread_tree(BT_test, chassis_executor, gimbal_executor);

	std::cout << "here" <<std::endl;

	//* 开启ros消息循环
	ros::spin();
	// ros::AsyncSpinner spinner(4);
	// spinner.start();

	std::cout << "over" << std::endl;
	
	ros::waitForShutdown();
	return 0;
}

/**
* ! 行为树测试
* ? 简单行为树
*/
void BT_test(ChassisExecutor* chassis_executor, GimbalExecutor* gimbal_executor){
	//*  
	//* 注册场地
	BehaviorTreeFactory factory;
	// Note: the name used to register should be the same used in the XML.

    // The recommended way to create a Node is through inheritance.
    // Even if it requires more boilerplate, it allows you to use more functionalities
    // like ports (we will discuss this in future tutorials).
    // factory.registerNodeType<ApproachObject>("ApproachObject");
	
	//done ------------ 注册节点们 ------------
	//* 移动到目标位置的节点
	factory.registerNodeType<MoveGoal>("MoveGoal");
	//* 射击动作节点（异步--父节点parallel）
	factory.registerNodeType<Shoot>("Shoot");
	//* 巡逻节点
	factory.registerNodeType<Patrol>("Patrol");
	//* 转身节点
	factory.registerNodeType<Turn>("Turn");
	//* 原地扭摆节点
	factory.registerNodeType<SwingIdle>("SwingIdle");
	//* 唤醒射击模块
	factory.registerNodeType<Awaken>("Awaken");
	//* 实时计算子弹发射数量
	factory.registerNodeType<ComputeShootNumber>("ComputeShootNumber");
	//* 从哨岗获取目标点
	factory.registerNodeType<FindAvailableShootPositionWithGuardMsg>("FindAvailableShootPositionWithGuardMsg");
	
	// done ---- PreConditions ----
	//* 比赛前五秒---用于判断是否开启摩擦轮
	factory.registerSimpleCondition("WhetherAwake", std::bind(Awake_Condition));
	//* 判断比赛是否开始
	factory.registerSimpleCondition("NotGameStage", std::bind(NotGameStage));
    //* 是否进行射击
	factory.registerSimpleCondition("WhetherFight", std::bind(Fight_Condition));
	//* 是否逃跑
	factory.registerSimpleCondition("WhetherEscape", std::bind(Escape_Condition));
	//* 是否进行巡逻
	factory.registerSimpleCondition("WhetherPatrol", std::bind(Patrol_Condition));
	//* 是否处于一个适合射击的位置
	factory.registerSimpleCondition("WhetherSuitableToFight", std::bind(Whether_Suitable_To_Fight));
	//* 加成区是否即将刷新
	factory.registerSimpleCondition("CheckAdditionGoingRefresh", std::bind(CheckAdditionGoingRefresh));
	//* 是否处于加成区
	factory.registerSimpleCondition("CheckPoseInAddition",std::bind(CheckPoseInAddition));
	//* 是否被攻击 然后能够开启原地扭转、摆动防御
	factory.registerSimpleCondition("WhetherTurnDefend", std::bind(TurnDefend_Condition));
	//* 是否可以原地摆动
	factory.registerSimpleCondition("WhetherSwingIdle", std::bind(Idle_Condition));
	//* 比赛结束，回到启动区
	factory.registerSimpleCondition("GameEnd", std::bind(GameEnd));
	//* 去子弹加成区
	factory.registerSimpleCondition("WhetherBulletsAddition", std::bind(BulletsAddition_Condition));
	//* 子弹加成区是否已经刷上
	factory.registerSimpleCondition("WhetherSuitableBulletsAddition", std::bind(Whether_Suitable_Bullets_Addition));
	//* 去血量加成区
	factory.registerSimpleCondition("WhetherHPAddition", std::bind(HPAddition_Condition));
	//* 血量加成区是否已经刷上
	factory.registerSimpleCondition("WhetherSuitableHPAddition", std::bind(Whether_Suitable_HP_Addition));
	//* 是否检测到装甲板
	factory.registerSimpleCondition("WhetherArmorDetected", std::bind(Whether_Armor_Detected));
	//* 是否连续检测到敌人的front姿势，采取绕后
	factory.registerSimpleCondition("WhetherSearchBack", std::bind(Whether_Search_Back));
	
	// done --------- 注册 功能函数 Functions --------
	//* 计算移动位置
	factory.registerSimpleAction("ComputeFightMoveGoal", std::bind(Compute_Fight_MoveGoal));
	//* 计算目标加成区
	factory.registerSimpleAction("ComputeAdditionGoal", std::bind(Compute_Addition_MoveGoal));
	//* 计算撤出加成/惩罚区的移动目的地
	factory.registerSimpleAction("ComputeWithdrawAdditionMoveGoal", std::bind(Compute_Withdraw_Addition_MoveGoal));
	//* 设置移动目标 --- blackboard_ptr->my_goal
	factory.registerSimpleAction("SetGoal", std::bind(SetGoal));
	//* 更新执行状态
	factory.registerSimpleAction("UpdateAdditionActiveState", std::bind(UpdateAdditionActiveState));
	//* 计算一次发射子弹的个数
	PortsList shoot_number_ports = { OutputPort<uint8_t>("shootnumber") };
	factory.registerSimpleAction("Compute_ShootNumber", Compute_ShootNumber, shoot_number_ports);
	//* 设置子弹加成区的移动目标
	//PortsList bullets_addition_goal_ports = { OutputPort<uint8_t>("bullets_addition_goal") };
	factory.registerSimpleAction("GetBulletsAdditionGoal", std::bind(GetBulletsAdditionGoal));
	//* 设置血量加成区的移动目标
	factory.registerSimpleAction("GetHPAdditionGoal", std::bind(GetHPAdditionGoal));
	//* 调整加成区位置确保刷上
	factory.registerSimpleAction("SetAdditionAdjustGoal", std::bind(SetAdditionAdjustGoal));
	//* 设置绕后状态
	factory.registerSimpleAction("SetSearchBackStatus", std::bind(SetSearchBackStatus));
	//* 取消绕后状态
	factory.registerSimpleAction("CancelSearchBackStatus", std::bind(CancelSearchBackStatus));

	//* 设置比赛结束
	factory.registerSimpleAction("SetGameEnd", std::bind(SetGameEnd));

	//* 创建行为树
	auto tree = factory.createTreeFromFile(bt_test_path);	//* 使用文件

	//* 初始化各个节点的私有变量
	for (auto& node : tree.nodes) {
		if (auto action_shoot = dynamic_cast<Shoot*>(node.get()))
		{
			action_shoot->init(gimbal_executor);
		}
		else if (auto action_movegoal = dynamic_cast<MoveGoal*>(node.get()))
		{
			//* 移动节点私有变量初始化
			action_movegoal->init(chassis_executor,gimbal_executor);
		}
		else if (auto action_patrol = dynamic_cast<Patrol*>(node.get()))
		{
			//* 巡逻节点初始化
			action_patrol->init(chassis_executor, gimbal_executor);
		}
		else if (auto action_turn = dynamic_cast<Turn*>(node.get()))
		{
			//* 转身节点初始化
			action_turn->init(chassis_executor, gimbal_executor);
		}
		else if (auto action_swingidle = dynamic_cast<SwingIdle*>(node.get()))
		{
			//* 原地扭摆节点初始化
			action_swingidle->init(chassis_executor,gimbal_executor);
		}
		
		else if (auto action_awaken = dynamic_cast<Awaken*>(node.get()))
		{
			action_awaken->init(gimbal_executor);
		}
	}

	int i = 0;

    std::cout << "bt run" << std::endl;	

	while(ros::ok()) {
		i = i+1;
		tree.tickRoot();
	}
	
	std::cout << "final test : " << i << std::endl;

	ros::waitForShutdown();
}
