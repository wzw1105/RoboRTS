#include <pthread.h>
#include <thread>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "./blackboard/blackboard.h"
#include "./action_node/FindAvailableShootPositionWithGuardMsg.h"
#include "./action_node/InitTheRobot.h"
#include "./action_node/MoveToBloodAddArea.h"
#include "./action_node/MoveToBootArea.h"
#include "./action_node/MoveToBulletsAddArea.h"
#include "./action_node/MoveToEnermyArea.h"
#include "./action_node/Patrol.h"
#include "./action_node/Shoot.h"
#include "./action_node/Swing360.h"

#include "./condition_node/ConditionNode.h"

using namespace roborts_decision;

const std::string behavior_tree_xml_path = ros::package::getPath("roborts_decision")+ "/behavior_tree/behavior_tree.xml";   // "/home/wangzw/roborts_ws/src/RoboRTS/decision/src/bt_test_files/bt_test.xml";

//BlackBoard* blackboard_ptr;
std::shared_ptr<BlackBoard> blackboard_ptr;
ChassisExecutor* chassis_executor;
GimbalExecutor* gimbal_executor;

void behavior_tree_process(){
	BT::BehaviorTreeFactory factory;

	std::cout << "Behavior Tree Start\n\n";
	
	// register action node
	factory.registerNodeType<FindAvailableShootPositionWithGuardMsg>("FindAvailableShootPositionWithGuardMsg");
	factory.registerNodeType<InitTheRobot>("InitTheRobot");
	factory.registerNodeType<MoveToBloodAddArea>("MoveToBloodAddArea");
	factory.registerNodeType<MoveToBootArea>("MoveToBootArea");
	factory.registerNodeType<MoveToBulletsAddArea>("MoveToBulletsAddArea");
	factory.registerNodeType<MoveToEnermyArea>("MoveToEnemyArea");
	factory.registerNodeType<Patrol>("Patrol");
	factory.registerNodeType<Shoot>("Shoot");
	factory.registerNodeType<Swing360>("Swing360");

	//register condition node
	ConditionNode ConditionNodeInterface(blackboard_ptr);
	factory.registerSimpleCondition("PreParation5Seconds", std::bind(&ConditionNode::PreParation5Seconds, &ConditionNodeInterface));
	factory.registerSimpleCondition("InGame", std::bind(&ConditionNode::InGame, &ConditionNodeInterface));
	factory.registerSimpleCondition("GameEnd", std::bind(&ConditionNode::GameEnd, &ConditionNodeInterface));
	factory.registerSimpleCondition("EnoughBlood", std::bind(&ConditionNode::EnoughBlood, &ConditionNodeInterface));
	factory.registerSimpleCondition("EmergencyBlood", std::bind(&ConditionNode::EmergencyBlood, &ConditionNodeInterface));
	factory.registerSimpleCondition("EnoughBullets", std::bind(&ConditionNode::EnoughBullets, &ConditionNodeInterface));
	factory.registerSimpleCondition("HaveBullets", std::bind(&ConditionNode::HaveBullets, &ConditionNodeInterface));
	factory.registerSimpleCondition("SufferFastHit", std::bind(&ConditionNode::SufferFastHit, &ConditionNodeInterface));
	factory.registerSimpleCondition("SufferMediumHit", std::bind(&ConditionNode::SufferMediumHit, &ConditionNodeInterface));
	factory.registerSimpleCondition("GuardMsg1SecondUseful", std::bind(&ConditionNode::GuardMsg1SecondUseful, &ConditionNodeInterface));
	factory.registerSimpleCondition("DetectedEnemyIn1Second", std::bind(&ConditionNode::DetectedEnemyIn1Second, &ConditionNodeInterface));
	factory.registerSimpleCondition("BulletsAddAreaAvaliable", std::bind(&ConditionNode::BulletsAddAreaAvaliable, &ConditionNodeInterface));
	factory.registerSimpleCondition("BloodAddAreaAvailable", std::bind(&ConditionNode::BloodAddAreaAvailable, &ConditionNodeInterface));
	std::cout << "register success.\n";

	//* 创建行为树
	auto tree = factory.createTreeFromFile(behavior_tree_xml_path);	//* 使用文件
	for (auto& node : tree.nodes) {
		if (auto action_shootaction = dynamic_cast<FindAvailableShootPositionWithGuardMsg*>(node.get())){
			std::cout << "guard register success.\n";
			action_shootaction->init(blackboard_ptr);
		}else if (auto action_movegoal = dynamic_cast<InitTheRobot*>(node.get())){
			std::cout << "init robot register success.\n";
			action_movegoal->init(blackboard_ptr, chassis_executor, gimbal_executor);
		}else if (auto action_patrol = dynamic_cast<MoveToBloodAddArea*>(node.get())){
			std::cout << "3\n";
			action_patrol->init(blackboard_ptr, chassis_executor);
		}else if (auto action_turn = dynamic_cast<MoveToBootArea*>(node.get())){
			std::cout << "4\n";
			action_turn->init(blackboard_ptr, chassis_executor);
		}else if (auto action_turn = dynamic_cast<MoveToBulletsAddArea*>(node.get())){
			std::cout << "5\n";
			action_turn->init(blackboard_ptr, chassis_executor);
		}else if (auto action_turn = dynamic_cast<MoveToEnermyArea*>(node.get())){
			std::cout << "6\n";
			action_turn->init(blackboard_ptr, chassis_executor);
		}else if (auto action_turn = dynamic_cast<Patrol*>(node.get())){
			std::cout << "7\n";
			action_turn->init(blackboard_ptr, chassis_executor);
		}else if (auto action_turn = dynamic_cast<Shoot*>(node.get())){
			std::cout << "8\n";
			action_turn->init(blackboard_ptr, gimbal_executor);
		}else if (auto action_turn = dynamic_cast<Swing360*>(node.get())){
			std::cout << "9\n";
			action_turn->init(blackboard_ptr, chassis_executor);
		}else{
			ROS_WARN("Don't have such node to init.");
		}
	}
	ROS_INFO("Start tick the tree.");
	while(ros::ok()) {
		tree.tickRoot();
	}
}

int main(int argc, char **argv) {
	//* ros 初始化
	ros::init(argc, argv, "tester");
	// blackboard_ptr = new BlackBoard();
	chassis_executor = new roborts_decision::ChassisExecutor;
	gimbal_executor = new roborts_decision::GimbalExecutor;
	blackboard_ptr = std::make_shared<BlackBoard>(gimbal_executor);
	//* 为行为树创建线程
	pthread_t tids;
	std::thread behaviortree_thread_(behavior_tree_process);
	std::cout << "here" <<std::endl;
	//* 开启ros消息循环
	ros::spin();
	std::cout << "over" << std::endl;
	pthread_exit(NULL);
	return 0;
}

