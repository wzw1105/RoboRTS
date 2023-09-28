/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include <csignal>

#include "global_planner_node.h"

namespace roborts_global_planner{

using roborts_common::ErrorCode;
using roborts_common::ErrorInfo;
using roborts_common::NodeState;
GlobalPlannerNode::GlobalPlannerNode() :
    new_path_(false),pause_(false), node_state_(NodeState::IDLE), error_info_(ErrorCode::OK),
    as_(nh_,"global_planner_node_action",boost::bind(&GlobalPlannerNode::GoalCallback,this,_1),false) {

  if (Init().IsOK()) {
    ROS_INFO("Global planner initialization completed.");
    StartPlanning();
    as_.start();
  } else {
    ROS_ERROR("Initialization failed.");
    SetNodeState(NodeState::FAILURE);
  }

}

ErrorInfo GlobalPlannerNode::Init() {

  // Load proto planning configuration parameters
  GlobalPlannerConfig global_planner_config;
  std::string full_path = ros::package::getPath("roborts_planning") + "/global_planner/config/global_planner_config.prototxt";
  if (!roborts_common::ReadProtoFromTextFile(full_path.c_str(),
                                           &global_planner_config)) {
    ROS_ERROR("Cannot load global planner protobuf configuration file.");
    return ErrorInfo(ErrorCode::GP_INITILIZATION_ERROR,
                     "Cannot load global planner protobuf configuration file.");
  }

  selected_algorithm_ = global_planner_config.selected_algorithm();
  cycle_duration_ = std::chrono::microseconds((int) (1e6 / global_planner_config.frequency()));
  max_retries_ = global_planner_config.max_retries();
  goal_distance_tolerance_ = global_planner_config.goal_distance_tolerance();
  goal_angle_tolerance_ = global_planner_config.goal_angle_tolerance();

  // ROS path visualize
  ros::NodeHandle viz_nh("~");
  path_pub_ = viz_nh.advertise<nav_msgs::Path>("path", 10);

  // Create tf listener
  tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

  // Create global costmap
  std::string map_path = ros::package::getPath("roborts_costmap") + \
      "/config/costmap_parameter_config_for_global_plan.prototxt";
  costmap_ptr_ = std::make_shared<roborts_costmap::CostmapInterface>("global_costmap",
                                                                           *tf_ptr_,
                                                                           map_path.c_str());
  // Create the instance of the selected algorithm
  global_planner_ptr_ = roborts_common::AlgorithmFactory<GlobalPlannerBase,CostmapPtr >::CreateAlgorithm(
      selected_algorithm_, costmap_ptr_);
  if (global_planner_ptr_== nullptr) {
    ROS_ERROR("global planner algorithm instance can't be loaded");
    return ErrorInfo(ErrorCode::GP_INITILIZATION_ERROR,
                     "global planner algorithm instance can't be loaded");
  }


  // Initialize path frame from global costmap
  path_.header.frame_id = costmap_ptr_->GetGlobalFrameID();
  return ErrorInfo(ErrorCode::OK);
}

void GlobalPlannerNode::GoalCallback(const roborts_msgs::GlobalPlannerGoal::ConstPtr &msg) {

  ROS_INFO("Received a Goal from client!");

  //Update current error and info
  ErrorInfo error_info = GetErrorInfo();
  NodeState node_state = GetNodeState();

  //Set the current goal
  SetGoal(msg->goal);

  //If the last state is not running, set it to running
  if (GetNodeState() != NodeState::RUNNING) {
    SetNodeState(NodeState::RUNNING);
  }

  //Notify the condition variable to stop lock waiting the fixed duration
  {
    std::unique_lock<std::mutex> plan_lock(plan_mutex_);
    plan_condition_.notify_one();
  }



  while (ros::ok()) {
    // Preempted and Canceled
    if (as_.isPreemptRequested()) {
      if (as_.isNewGoalAvailable()) {
        as_.setPreempted();
        ROS_INFO("Override!");
        break;
      }else{
        as_.setPreempted();
        SetNodeState(NodeState::IDLE);
        ROS_INFO("Cancel!");
        break;
      }
    }

    // Update the current state and error info
    node_state = GetNodeState();
    error_info = GetErrorInfo();
    //TODO: seem useless to check state here, as it would never be IDLE state
    if(node_state == NodeState::RUNNING || node_state == NodeState::SUCCESS || node_state == NodeState::FAILURE) {
      roborts_msgs::GlobalPlannerFeedback feedback;
      roborts_msgs::GlobalPlannerResult result;
      // If error occurs or planner produce new path, publish the feedback
      if (!error_info.IsOK() || new_path_) {
        if (!error_info.IsOK()) {
          feedback.error_code = error_info.error_code();
          feedback.error_msg = error_info.error_msg();
          SetErrorInfo(ErrorInfo::OK());
        }
        if (new_path_) {
          feedback.path = path_;
          new_path_ = false;
        }
        as_.publishFeedback(feedback);
      }

      // After get the result, deal with actionlib server and jump out of the loop
      if(node_state == NodeState::SUCCESS){
        result.error_code = error_info.error_code();
        as_.setSucceeded(result,error_info.error_msg());
        SetNodeState(NodeState::IDLE);
        break;
      }
      else if(node_state == NodeState::FAILURE){
        result.error_code = error_info.error_code();
        as_.setAborted(result,error_info.error_msg());
        SetNodeState(NodeState::IDLE);
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  }

}

NodeState GlobalPlannerNode::GetNodeState() {
  std::lock_guard<std::mutex> node_state_lock(node_state_mtx_);
  return node_state_;
}

void GlobalPlannerNode::SetNodeState(NodeState node_state) {
  std::lock_guard<std::mutex> node_state_lock(node_state_mtx_);
  node_state_ = node_state;
}

ErrorInfo GlobalPlannerNode::GetErrorInfo() {
  std::lock_guard<std::mutex> error_info_lock(error_info_mtx_);
  return error_info_;
}

void GlobalPlannerNode::SetErrorInfo(ErrorInfo error_info) {
  std::lock_guard<std::mutex> node_state_lock(error_info_mtx_);
  error_info_ = error_info;
}

geometry_msgs::PoseStamped GlobalPlannerNode::GetGoal() {
  std::lock_guard<std::mutex> goal_lock(goal_mtx_);
  return goal_;
}

void GlobalPlannerNode::SetGoal(geometry_msgs::PoseStamped goal) {
  std::lock_guard<std::mutex> goal_lock(goal_mtx_);
  goal_ = goal;
}

void GlobalPlannerNode::StartPlanning() {
  SetNodeState(NodeState::IDLE);
  plan_thread_ = std::thread(&GlobalPlannerNode::PlanThread, this);
}

void GlobalPlannerNode::StopPlanning() {
  SetNodeState(NodeState::RUNNING);
  if (plan_thread_.joinable()) {
    plan_thread_.join();
  }
}

void GlobalPlannerNode::PlanThread() {
  ROS_INFO("Plan thread start!");
  geometry_msgs::PoseStamped current_start;
  geometry_msgs::PoseStamped current_goal;
  std::vector<geometry_msgs::PoseStamped> current_path;
  std::chrono::microseconds sleep_time = std::chrono::microseconds(0);
  ErrorInfo error_info;
  int retries = 0;
  while (ros::ok()) {
    ROS_INFO("Wait to plan!");
    std::unique_lock<std::mutex> plan_lock(plan_mutex_);
    plan_condition_.wait_for(plan_lock, sleep_time);
    while (GetNodeState()!=NodeState::RUNNING){
      std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
    ROS_INFO("Go on planning!");

    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

    {
      std::unique_lock<roborts_costmap::Costmap2D::mutex_t> lock(*(costmap_ptr_->GetCostMap()->GetMutex()));
      bool error_set = false;
      //Get the robot current pose
      while (!costmap_ptr_->GetRobotPose(current_start)) {
        if (!error_set) {
          ROS_ERROR("Get Robot Pose Error.");
          SetErrorInfo(ErrorInfo(ErrorCode::GP_GET_POSE_ERROR, "Get Robot Pose Error."));
          error_set = true;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(1));
      }

      //Get the robot current goal and transform to the global frame
      current_goal = GetGoal();

      if (current_goal.header.frame_id != costmap_ptr_->GetGlobalFrameID()) {
        current_goal = costmap_ptr_->Pose2GlobalFrame(current_goal);
        SetGoal(current_goal);
      }

      //Plan
      error_info = global_planner_ptr_->Plan(current_start, current_goal, current_path);


    }

    if (error_info.IsOK()) {
      //When planner succeed, reset the retry times
      retries = 0;
      PathVisualization(current_path);

      //Set the goal to avoid the same goal from getting transformed every time
      current_goal = current_path.back();
      SetGoal(current_goal);

      //Decide whether robot reaches the goal according to tolerance
      if (GetDistance(current_start, current_goal) < goal_distance_tolerance_
          && GetAngle(current_start, current_goal) < goal_angle_tolerance_
          ) {
        SetNodeState(NodeState::SUCCESS);
      }
    } else if (max_retries_ > 0 && retries > max_retries_) {
      //When plan failed to max retries, return failure
      ROS_ERROR("Can not get plan with max retries( %d )", max_retries_ );
      error_info = ErrorInfo(ErrorCode::GP_MAX_RETRIES_FAILURE, "Over max retries.");
      SetNodeState(NodeState::FAILURE);
      retries=0;
    } else if (error_info == ErrorInfo(ErrorCode::GP_GOAL_INVALID_ERROR)){
      //When goal is not reachable, return failure immediately
      ROS_ERROR("Current goal is not valid!");
      SetNodeState(NodeState::FAILURE);
      retries=0;
    }
    else {
      //Increase retries
      retries++;
      ROS_ERROR("Can not get plan for once. %s", error_info.error_msg().c_str());
    }
    // Set and update the error info
    SetErrorInfo(error_info);

    // Deal with the duration to wait
    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
    std::chrono::microseconds execution_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    sleep_time = cycle_duration_ - execution_duration;

    // Report warning while planning timeout
    if (sleep_time <= std::chrono::microseconds(0)) {
      ROS_ERROR("The time planning once is %ld beyond the expected time %ld",
                execution_duration.count(),
                cycle_duration_.count());
      sleep_time = std::chrono::microseconds(0);
      SetErrorInfo(ErrorInfo(ErrorCode::GP_TIME_OUT_ERROR, "Planning once time out."));
    }
  }


  ROS_INFO("Plan thread terminated!");
}

void GlobalPlannerNode::PathVisualization(const std::vector<geometry_msgs::PoseStamped> &path) {
  //添加floyd平滑
  std::vector<geometry_msgs::PoseStamped> p_=FloydSmooth(path);
  
  path_.poses = p_;
  path_pub_.publish(path_);
  new_path_ = true;
}

double GlobalPlannerNode::GetDistance(const geometry_msgs::PoseStamped &pose1,
                                      const geometry_msgs::PoseStamped &pose2) {
  const geometry_msgs::Point point1 = pose1.pose.position;
  const geometry_msgs::Point point2 = pose2.pose.position;
  const double dx = point1.x - point2.x;
  const double dy = point1.y - point2.y;
  return std::sqrt(dx * dx + dy * dy);
}

double GlobalPlannerNode::GetAngle(const geometry_msgs::PoseStamped &pose1,
                                   const geometry_msgs::PoseStamped &pose2) {
  const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
  const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
  tf::Quaternion rot1, rot2;
  tf::quaternionMsgToTF(quaternion1, rot1);
  tf::quaternionMsgToTF(quaternion2, rot2);
  return rot1.angleShortestPath(rot2);
}

GlobalPlannerNode::~GlobalPlannerNode() {
  StopPlanning();
}
//添加
std::vector<geometry_msgs::PoseStamped> GlobalPlannerNode::FloydSmooth( std::vector<geometry_msgs::PoseStamped> path){
  int pathlength=path.size();
  std::vector<geometry_msgs::PoseStamped> newpath;
  newpath.push_back(path[0]);
  //去除共线点
  for (int i = 1; i < pathlength - 1; i++){
    if(((path[i].pose.position.x - path[i-1].pose.position.x) == (path[i+1].pose.position.x - path[i].pose.position.x)) && (path[i].pose.position.y - path[i-1].pose.position.y) == (path[i+1].pose.position.y - path[i].pose.position.y)){
        i++;
        //path.erase(path.begin()+i+1);
      }
    else {
      newpath.push_back(path[i]);
    }
  }
  newpath.push_back(path[pathlength-1]);

  //去除多余拐点
  std::vector<geometry_msgs::PoseStamped> p;
  int n = newpath.size();
  for(int i = 0; i <= n - 1; i++){
    for(int j = n - 1; j >= i + 1; j--){
      if(CheckCross(newpath[i], newpath[j])){//如果中间没有障碍物
        // 删除中间的点，仅保留两端的
        p.push_back(newpath[i]);
        i = j;
        break;
      }
    }
    p.push_back(newpath[i]);
  }

  n = p.size();
  // newpath.clear();
  std::vector<geometry_msgs::PoseStamped> result_path;
  for(int i = 0; i < n - 1; i++) AddPoint(result_path, p[i], p[i + 1]);
  result_path.push_back(p[n - 1]);
  return result_path;
}

double GlobalPlannerNode::GetNextY(double x, double y, double k) {
  int nextx = (int) (x + 1.01);
  return y + (nextx - x) * k;
}

bool GlobalPlannerNode::CheckVerticalLineCost(int x, int y_1, int y_2) {
  roborts_costmap::Costmap2D* costmap_ = costmap_ptr_ -> GetCostMap();
  if(y_1 <= y_2) {for(int y_ = y_1; y_ <= y_2; y_++) if(costmap_ptr_ -> GetCostMap() -> GetCost(x, y_) >= roborts_costmap::INSCRIBED_INFLATED_OBSTACLE) return false;}
  else {for(int y_ = y_1; y_ >= y_2; y_--) if(costmap_ptr_ -> GetCostMap() -> GetCost(x, y_) >= roborts_costmap::INSCRIBED_INFLATED_OBSTACLE) return false;}
  return true;
}

bool GlobalPlannerNode::CheckCross(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped end) {
  //std::cout << "the cost in(1.9, 2.2): " << (int)(costmap_ptr_->GetCostMap()->GetCost(124, 40)) << "\n";
  unsigned int startx, starty, endx, endy;
  if(!costmap_ptr_->GetCostMap()->World2Map(start.pose.position.x, start.pose.position.y, startx, starty)) return false;
  if(!costmap_ptr_->GetCostMap()->World2Map(end.pose.position.x, end.pose.position.y, endx, endy)) return false;
  //std::cout << "startx, starty, endx, endy = " << startx << " " << starty << " " << endx << " " << endy << "\n";
  if(startx > endx) std::swap(startx, endx), std::swap(starty, endy);
  if(startx == endx) {
    for(int y_ = starty; y_ <= endy; y_++){
      if(costmap_ptr_->GetCostMap()->GetCost(startx, y_) >= roborts_costmap::INSCRIBED_INFLATED_OBSTACLE) return false;
    }
    return true;
  }else {
    double k = (double)((int)endy - (int)starty) / ((int)endx - (int)startx);
    //std::cout << "startx, starty, nexty, k = " << startx << " " << starty << " " <<  (int)(GetNextY(startx + 0.5, starty + 0.5, k)) + (starty < endy ? 1 : 0) << " " << k << "\n";
    if(!CheckVerticalLineCost(startx, starty, (int)(GetNextY(startx + 0.5, starty + 0.5, k)) )) return false;
    double prev_y =  GetNextY(startx + 0.5, starty + 0.5, k);
    for(int x_ = startx + 1; x_ < endx; x_++) {
      double next_y = GetNextY(x_, prev_y, k);
      if(!CheckVerticalLineCost(x_, (int)prev_y, (int)next_y + (starty < endy ? 1 : 0))) return false;
      //std::cout << "x_, prev_y, next_y = " << x_ << " " << (int)prev_y << " " <<  (int)next_y + (starty < endy ? 1 : 0) << std::endl;
      prev_y = next_y;
    }
    if(!CheckVerticalLineCost(endx, prev_y, endy)) return false;
    //std::cout << "endx, prev_y, endy = " << endx << " " << (int)prev_y << " " <<  endy << std::endl;
    //std::cout << "\n\n";
    return true;
  }
}

void GlobalPlannerNode::AddPoint(std::vector<geometry_msgs::PoseStamped> &path, geometry_msgs::PoseStamped &start, geometry_msgs::PoseStamped &end) {
  unsigned int startx, starty, endx, endy;
  costmap_ptr_->GetCostMap()->World2Map(start.pose.position.x, start.pose.position.y, startx, starty);
  costmap_ptr_->GetCostMap()->World2Map(end.pose.position.x, end.pose.position.y, endx, endy);
  std::cout << "startx, starty, endx, endy = " << startx << " " << starty << " " << endx << " " << endy << "\n";
  geometry_msgs::PoseStamped new_p;
  new_p.pose.orientation.w = 1;
  new_p.header.frame_id = "map";
  int x_inc = startx < endx ? 1 : -1, y_inc = starty < endy ? 1 : -1;
  if(startx == endx) {
    for(int y_ = starty; y_ != endy; y_ += y_inc) {
      if((y_inc > 0 && y_ > endy) || (y_inc < 0 && y_ < endy)) break;
      costmap_ptr_->GetCostMap()->Map2World(startx, y_, new_p.pose.position.x, new_p.pose.position.y);
      path.push_back(new_p);
    }
  }else{
    double k = (double)((int)endy - (int)starty) / ((int)endx - (int)startx);

    //for first column
    double nexty = starty + 0.5 + (double)x_inc * 0.5 * k, prevy = starty;
    for(int y_ = starty; y_ != (int)nexty; y_ += y_inc) {
      if((y_inc > 0 && y_ > nexty) || (y_inc < 0 && y_ < nexty)) break;
      costmap_ptr_->GetCostMap()->Map2World(startx, y_, new_p.pose.position.x, new_p.pose.position.y);
      path.push_back(new_p);
    } 

    //for column between the first and last
    for(int x_ = startx + x_inc; x_ != (int)endx; x_ += x_inc) {
      if((x_inc > 0 && x_ > endx) || (x_inc < 0 && x_ < endx)) break;
      prevy = nexty, nexty = prevy + (double)x_inc * k;
      for(int y_ = (int)prevy; y_ != (int)nexty + y_inc; y_ += y_inc) {
        if((y_inc > 0 && y_ > nexty + y_inc) || (y_inc < 0 && y_ < nexty + y_inc)) break;
        costmap_ptr_->GetCostMap()->Map2World(x_, y_, new_p.pose.position.x, new_p.pose.position.y);
        path.push_back(new_p);  
      }
    }

    //for last column
    prevy = nexty, nexty = endy;
    for(int y_ = prevy; y_ != endy; y_+= y_inc) {
      if((y_inc > 0 && y_ > endy) || (y_inc < 0 && y_ < endy)) break;
      costmap_ptr_->GetCostMap()->Map2World(endx, y_, new_p.pose.position.x, new_p.pose.position.y);
      path.push_back(new_p);
    }
  }
}


} //namespace roborts_global_planner

int main(int argc, char **argv) {

  ros::init(argc, argv, "global_planner_node");
  roborts_global_planner::GlobalPlannerNode global_planner;
  ros::spin();
  return 0;
}
