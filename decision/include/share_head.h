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
#ifndef _SHARE_HEAD_H_
#define _SHARE_HEAD_H_

#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <iostream>
#include <math.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/terminal_state.h>	//action服务
#include <actionlib/client/simple_action_client.h>
#include <roborts_msgs/ShootCmd.h>				//消息头文件
#include <roborts_msgs/ArmorDetectionAction.h>	//目标检测 action 头文件
#include <roborts_msgs/GimbalAngle.h>			//云台
#include <roborts_msgs/TwistAccel.h>    //* 加速度控制器

#include "../src/blackboard/blackboard.h"
#include "../src/line_iterator.h"
#include "../src/executor/chassis_executor.h"
#include "../src/executor/gimbal_executor.h"

// extern BlackBoard* blackboard;

extern BlackBoard::Ptr blackboard_ptr;


#endif