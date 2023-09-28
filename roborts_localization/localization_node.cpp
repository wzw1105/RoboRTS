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


#include "localization_node.h"

namespace roborts_localization{

LocalizationNode::LocalizationNode(std::string name) {
  CHECK(Init()) << "Module "  << name <<" initialized failed!";
  initialized_ = true;
}


bool LocalizationNode::Init() {

  LocalizationConfig localization_config;
  localization_config.GetParam(&nh_);

  odom_frame_   = std::move(localization_config.odom_frame_id);
  global_frame_ = std::move(localization_config.global_frame_id);
  base_frame_   = std::move(localization_config.base_frame_id);

  laser_topic_ = std::move(localization_config.laser_topic_name);

  init_pose_ = {localization_config.initial_pose_x,
                localization_config.initial_pose_y,
                localization_config.initial_pose_a};
  init_cov_ = {localization_config.initial_cov_xx,
               localization_config.initial_cov_yy,
               localization_config.initial_cov_aa};

  transform_tolerance_  = ros::Duration(localization_config.transform_tolerance);
  publish_visualize_ = localization_config.publish_visualize;
  // 添加
  time_range=localization_config.time_range;
  dis_range=localization_config.dis_range;
  fusion_time_start_ = localization_config.fusion_time_start;
  fusion_time_end_ = localization_config.fusion_time_end;

  tf_broadcaster_ptr_ = std::make_unique<tf::TransformBroadcaster>();
  tf_listener_ptr_ = std::make_unique<tf::TransformListener>();

  distance_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("distance_map", 1, true);
  particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2, true);

  // Use message filter for time synchronizer (laser scan topic and tf between odom and base frame)
  laser_scan_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::LaserScan>>(nh_, laser_topic_, 100);
  laser_scan_filter_ = std::make_unique<tf::MessageFilter<sensor_msgs::LaserScan>>(*laser_scan_sub_,
                                                                                   *tf_listener_ptr_,
                                                                                   odom_frame_,
                                                                                   100);
  laser_scan_filter_->registerCallback(boost::bind(&LocalizationNode::LaserScanCallback, this, _1));

  initial_pose_sub_ = nh_.subscribe("initialpose", 2, &LocalizationNode::InitialPoseCallback, this);
  
  //添加add subscriber
  pose_from_post_sub=nh_.subscribe("pose_from_post",2,&LocalizationNode::PosefromPost,this);
  // //* 订阅裁判系统发布的ID
  this->referee_robot_id_sub_ = this->nh_.subscribe("robot_status",2,&LocalizationNode::CallBack_Sub_Referee_ID,this);
  // //* 订阅裁判系统比赛阶段消息（包含 比赛剩余时间
  this->referee_game_status_sub_ = this->nh_.subscribe("game_status",2,&LocalizationNode::CallBack_Sub_Referee_Game_Status,this);
  // //* 初始化ID = 0
  this->my_ID = 0;
  pose_from_post.header.frame_id="map";
  pose_from_post.header.stamp=ros::Time::now();
  pose_from_post.pose.position.x=10.0;
  pose_from_post.pose.position.y=10.0;
  // remaining_time_ = 165;
  // game_status_ = 4;

  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("amcl_pose", 2, true);

  amcl_ptr_= std::make_unique<Amcl>();
  amcl_ptr_->GetParamFromRos(&nh_);
  amcl_ptr_->Init(init_pose_, init_cov_);

  map_init_ = GetStaticMap();
  laser_init_ = GetLaserPose();

  return map_init_&&laser_init_;
}

bool LocalizationNode::GetStaticMap(){
  static_map_srv_ = nh_.serviceClient<nav_msgs::GetMap>("static_map");
  ros::service::waitForService("static_map", -1);
  nav_msgs::GetMap::Request req;
  nav_msgs::GetMap::Response res;
  if(static_map_srv_.call(req,res)) {
    LOG_INFO << "Received Static Map";
    amcl_ptr_->HandleMapMessage(res.map, init_pose_, init_cov_);
    first_map_received_ = true;
    return true;
  } else{
    LOG_ERROR << "Get static map failed";
    return false;
  }
}

bool LocalizationNode::GetLaserPose() {
  auto laser_scan_msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>(laser_topic_);

  Vec3d laser_pose;
  laser_pose.setZero();
  GetPoseFromTf(base_frame_, laser_scan_msg->header.frame_id, ros::Time(), laser_pose);
  laser_pose[2] = 0; // No need for rotation, or will be error
  DLOG_INFO << "Received laser's pose wrt robot: "<<
            laser_pose[0] << ", " <<
            laser_pose[1] << ", " <<
            laser_pose[2];

  amcl_ptr_->SetLaserSensorPose(laser_pose);
  return true;
}

void LocalizationNode::InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &init_pose_msg) {

  if (init_pose_msg->header.frame_id == "") {
    LOG_WARNING << "Received initial pose with empty frame_id.";
  } // Only accept initial pose estimates in the global frame
  else if (tf_listener_ptr_->resolve(init_pose_msg->header.frame_id) !=
           tf_listener_ptr_->resolve(global_frame_)) {
    LOG_ERROR << "Ignoring initial pose in frame \" "
              << init_pose_msg->header.frame_id
              << "\"; initial poses must be in the global frame, \""
              << global_frame_;
    return;
  }

  // In case the client sent a pose estimate in the past, integrate the
  // intervening odometric change.
  tf::StampedTransform tx_odom;
  try {
    ros::Time now = ros::Time::now();
    tf_listener_ptr_->waitForTransform(base_frame_,
                                       init_pose_msg->header.stamp,
                                       base_frame_,
                                       now,
                                       odom_frame_,
                                       ros::Duration(0.5));
    tf_listener_ptr_->lookupTransform(base_frame_,
                                      init_pose_msg->header.stamp,
                                      base_frame_,
                                      now,
                                      odom_frame_, tx_odom);
  }
  catch (tf::TransformException &e) {
    tx_odom.setIdentity();
  }
  tf::Pose pose_new;
  tf::Pose pose_old;
  tf::poseMsgToTF(init_pose_msg->pose.pose, pose_old);
  pose_new = pose_old * tx_odom;

  // Transform into the global frame
  DLOG_INFO << "Setting pose " << ros::Time::now().toSec() << ", "
            << pose_new.getOrigin().x() << ", " << pose_new.getOrigin().y();

  Vec3d init_pose_mean;
  Mat3d init_pose_cov;
  init_pose_mean.setZero();
  init_pose_cov.setZero();
  double yaw, pitch, roll;
  init_pose_mean(0) = pose_new.getOrigin().x();
  init_pose_mean(1) = pose_new.getOrigin().y();
  pose_new.getBasis().getEulerYPR(yaw, pitch, roll);
  init_pose_mean(2) = yaw;
  init_pose_cov = math::MsgCovarianceToMat3d(init_pose_msg->pose.covariance);

  amcl_ptr_->HandleInitialPoseMessage(init_pose_mean, init_pose_cov);
}



void LocalizationNode::LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &laser_scan_msg_ptr){


  //std::lock_guard<std::mutex> receive_lock(receive_mtx_); //添加锁
  last_laser_msg_timestamp_ = laser_scan_msg_ptr->header.stamp;

  Vec3d pose_in_odom;
  if(!GetPoseFromTf(odom_frame_, base_frame_, last_laser_msg_timestamp_, pose_in_odom))
  {
    LOG_ERROR << "Couldn't determine robot's pose";
    return;
  }

  double angle_min = 0 , angle_increment = 0;
  sensor_msgs::LaserScan laser_scan_msg = *laser_scan_msg_ptr;
  TransformLaserscanToBaseFrame(angle_min, angle_increment, laser_scan_msg);

  if(game_status_ == 4 && remaining_time_ > fusion_time_end_ && remaining_time_ < fusion_time_start_){
    // ROS_INFO("start using position from guard");
    if(pose_from_post.pose.position.x < 10.0){
      //进行条件判断
      double cal_distance=0.04;
      int cal_time=0.0;//std::numeric_limits<float>::infinity();
      ros::Time current_time = ros::Time::now();
      cal_time=(current_time.sec-pose_from_post.header.stamp.sec)*1e+9+current_time.nsec-pose_from_post.header.stamp.nsec;
      cal_distance=pow(pose_from_post.pose.position.x-hyp_pose_.pose_mean[0],2.0)+pow(pose_from_post.pose.position.y-hyp_pose_.pose_mean[1],2.0);
      //ROS_INFO(" %f \n", pow(cal_distance,0.5));
      if((pow(cal_distance,0.5)> dis_range)&&(cal_time<time_range)){ //1m范围内，15ms范围内
        //std::lock_guard<std::mutex> receive_lock(receive_mtx_); //添加锁
        ROS_INFO("TRUE!!!!");
        pose_received=true; 
      }
      else { 
        pose_received = false;
      }
    }
    else{
      pose_received = false;
    }
  }
  else {
    pose_received = false;
  }
  
  amcl_ptr_->Update(pose_in_odom,
                  laser_scan_msg,
                  angle_min,
                  angle_increment,
                  particlecloud_msg_,
                  hyp_pose_,
                  pose_received,//添加
                  pose_from_post //添加
                  );
 
  LOG_ERROR_IF(!PublishTf()) << "Publish Tf Error!";

  if(publish_visualize_){
    PublishVisualize();
  }

}

void LocalizationNode::PublishVisualize(){

  if(pose_pub_.getNumSubscribers() > 0){
    pose_msg_.header.stamp = ros::Time::now();
    pose_msg_.header.frame_id = global_frame_;
    pose_msg_.pose.position.x = hyp_pose_.pose_mean[0];
    pose_msg_.pose.position.y = hyp_pose_.pose_mean[1];
    pose_msg_.pose.orientation = tf::createQuaternionMsgFromYaw(hyp_pose_.pose_mean[2]);
    pose_pub_.publish(pose_msg_);
  }

  if(particlecloud_pub_.getNumSubscribers() > 0){
    particlecloud_msg_.header.stamp = ros::Time::now();
    particlecloud_msg_.header.frame_id = global_frame_;
    particlecloud_pub_.publish(particlecloud_msg_);
  }

  if(!publish_first_distance_map_) {
    distance_map_pub_.publish(amcl_ptr_->GetDistanceMapMsg());
    publish_first_distance_map_ = true;
  }
}

bool LocalizationNode::PublishTf() {
  ros::Time transform_expiration = (last_laser_msg_timestamp_ + transform_tolerance_);
  if (amcl_ptr_->CheckTfUpdate()) {
    // Subtracting base to odom from map to base and send map to odom instead
    tf::Stamped<tf::Pose> odom_to_map;
    try {
      tf::Transform tmp_tf(tf::createQuaternionFromYaw(hyp_pose_.pose_mean[2]),
                           tf::Vector3(hyp_pose_.pose_mean[0],
                                       hyp_pose_.pose_mean[1],
                                       0.0));
      tf::Stamped<tf::Pose> tmp_tf_stamped(tmp_tf.inverse(),
                                           last_laser_msg_timestamp_,
                                           base_frame_);
      this->tf_listener_ptr_->transformPose(odom_frame_,
                                            tmp_tf_stamped,
                                            odom_to_map);
    } catch (tf::TransformException &e) {
      LOG_ERROR << "Failed to subtract base to odom transform" << e.what();
      return false;
    }

    latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                               tf::Point(odom_to_map.getOrigin()));
    latest_tf_valid_ = true;

    tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                        transform_expiration,
                                        global_frame_,
                                        odom_frame_);
    this->tf_broadcaster_ptr_->sendTransform(tmp_tf_stamped);
    sent_first_transform_ = true;
    return true;
  } else if (latest_tf_valid_) {
    // Nothing changed, so we'll just republish the last transform
    tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                        transform_expiration,
                                        global_frame_,
                                        odom_frame_);
    this->tf_broadcaster_ptr_->sendTransform(tmp_tf_stamped);
    return true;
  }
  else{
    return false;
  }
}

bool LocalizationNode::GetPoseFromTf(const std::string &target_frame,
                   const std::string &source_frame,
                   const ros::Time &timestamp,
                   Vec3d &pose)
{
  tf::Stamped<tf::Pose> ident(tf::Transform(tf::createIdentityQuaternion(),
                                            tf::Vector3(0, 0, 0)),
                              timestamp,
                              source_frame);
  tf::Stamped<tf::Pose> pose_stamp;
  try {
    this->tf_listener_ptr_->transformPose(target_frame,
                                          ident,
                                          pose_stamp);
  } catch (tf::TransformException &e) {
    LOG_ERROR << "Couldn't transform from "
              << source_frame
              << "to "
              << target_frame;
    return false;
  }

  pose.setZero();
  pose[0] = pose_stamp.getOrigin().x();
  pose[1] = pose_stamp.getOrigin().y();
  double yaw,pitch, roll;
  pose_stamp.getBasis().getEulerYPR(yaw, pitch, roll);
  pose[2] = yaw;

  return true;
}

void LocalizationNode::TransformLaserscanToBaseFrame(double &angle_min,
                                                     double &angle_increment,
                                                     const sensor_msgs::LaserScan &laser_scan_msg) {

  // To account for lasers that are mounted upside-down, we determine the
  // min, max, and increment angles of the laser in the base frame.
  // Construct min and max angles of laser, in the base_link frame.
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, laser_scan_msg.angle_min);
  tf::Stamped<tf::Quaternion> min_q(q, laser_scan_msg.header.stamp,
                                    laser_scan_msg.header.frame_id);
  q.setRPY(0.0, 0.0, laser_scan_msg.angle_min
      + laser_scan_msg.angle_increment);
  tf::Stamped<tf::Quaternion> inc_q(q, laser_scan_msg.header.stamp,
                                    laser_scan_msg.header.frame_id);

  try {
    tf_listener_ptr_->transformQuaternion(base_frame_,
                                          min_q,
                                          min_q);
    tf_listener_ptr_->transformQuaternion(base_frame_,
                                          inc_q,
                                          inc_q);
  }
  catch (tf::TransformException &e) {
    LOG_WARNING << "Unable to transform min/max laser angles into base frame: " << e.what();
    return;
  }

  angle_min = tf::getYaw(min_q);
  angle_increment = (tf::getYaw(inc_q) - angle_min);

  // Wrapping angle to [-pi .. pi]
  angle_increment = (std::fmod(angle_increment + 5 * M_PI, 2 * M_PI) - M_PI);

}



//添加一个接收器的回调函数
void LocalizationNode::PosefromPost(const roborts_msgs::CarMsgs::ConstPtr &pose_from_post_msg){
  //保留接收数据
  pose_from_post.header.frame_id="map";
  pose_from_post.header.stamp=pose_from_post_msg->stamp_guard;
  int robot_index=0;

  for (int i=0;i<4;i++){
    int id =pose_from_post_msg->car_msgs[i].id;
    int color = pose_from_post_msg->car_msgs[i].color;
    if((my_ID == 1 && id == 1 && color == 'r')||(my_ID == 2 && id == 2 && color == 'r')||
      (my_ID == 101 && id == 1 && color == 'b')||(my_ID == 102 && id == 2 && color == 'b'))
    {
      robot_index = i;
      break;
    }
  }

  // if(my_ID==1 ){
  //   robot_index=0;
  // }
  // else if(my_ID==2){
  //   robot_index=1;
  // }
  // else if(my_ID==101){
  //   robot_index=2;
  // }
  // else if (my_ID==102){
  //   robot_index=3;
  // }
  // else {
  //   ROS_INFO("my id is error!\n");
  //   return ;
  // }
  // pose_from_post.pose.position.x=pose_from_post_msg->car_msgs[robot_index].x;//
  // pose_from_post.pose.position.y=pose_from_post_msg->car_msgs[robot_index].y;
  if(pose_from_post_msg->car_msgs[robot_index].id != 0){
    pose_from_post.pose.position.x=pose_from_post_msg->car_msgs[robot_index].x;//
    pose_from_post.pose.position.y=pose_from_post_msg->car_msgs[robot_index].y;

  }
  else {
    pose_from_post.pose.position.x=10.0;
    pose_from_post.pose.position.y=10.0;
  }

  /*
  //进行条件判断
  double cal_distance=0.0;
  int time_dis=0;
  ros::Time current_time = ros::Time::now();
  time_dis=(current_time.sec-pose_from_post_msg->stamp_guard.sec)*1e+9+current_time.nsec-pose_from_post_msg->stamp_guard.nsec;
  cal_distance=pow(pose_from_post.pose.position.x-hyp_pose_.pose_mean[0],2.0)+pow(pose_from_post.pose.position.y-hyp_pose_.pose_mean[1],2.0);

  if((pow(cal_distance,0.5)>1.0)&&(time_dis<15000000)){ //1m范围内，15ms范围内
    std::lock_guard<std::mutex> receive_lock(receive_mtx_); //添加锁
    pose_received=true;
  }
  else { 
    pose_received = false;
  }
  
  ROS_INFO("Localization_node received msgs!\n");
  */

  return ;
}

void LocalizationNode::CallBack_Sub_Referee_ID(const roborts_msgs::RobotStatus::ConstPtr& robot_status){
  this->my_ID = robot_status->id;
}

void LocalizationNode::CallBack_Sub_Referee_Game_Status(const roborts_msgs::GameStatus::ConstPtr& game_status){
  this->game_status_ = game_status->game_status;
  this->remaining_time_ =(int) game_status->remaining_time;
}


}// roborts_localization

int main(int argc, char **argv) {
  roborts_localization::GLogWrapper glog_wrapper(argv[0]);
  ros::init(argc, argv, "localization_node");
  roborts_localization::LocalizationNode localization_node("localization_node");
  ros::AsyncSpinner async_spinner(THREAD_NUM);
  async_spinner.start();
  ros::waitForShutdown();
  return 0;
}

