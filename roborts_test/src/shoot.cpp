#include "shoot.h"

Shoot::Shoot() {
  tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
  std::string map_path = ros::package::getPath("roborts_costmap") + "/config/costmap_parameter_config_for_decision.prototxt";
  costmap_ptr_ = std::make_shared<roborts_costmap::CostmapInterface>("decision_costmap", *tf_ptr_, map_path);
  //nh_.param<bool>("enable_shoot", enable_shoot, false);
  enable_shoot = true;
  nh_.param<float>("min_area_for_resample", min_area_for_resample, 800);
  nh_.param<float>("min_dis_for_resample", min_dis_for_resample, 0.4);
  nh_.param<int>("bullet_min_cell_dis", bullet_min_cell_dis, 1);
  nh_.param<float>("yaw_change_threshold", yaw_change_threshold, 0.01);
  nh_.param<float>("pitch_change_threshold", pitch_change_threshold, 0.01);
  nh_.param<int>("enermy_color_", enermy_color_, 1);
  nh_.param<float>("h", h, 0.1);
  nh_.param<float>("init_k_", init_k_, 0.026); //offset_x
  nh_.param<float>("offset_x", offset_x, 0.026);
  nh_.param<int>("update_step", update_step, 10);

  have_last_armor_ = false;
  ShootInit();
}

Shoot::~Shoot() {
  fricCtrl.request.open = false;
  if(fric_client.call(fricCtrl)) {
    ROS_INFO("Close Fric successfully.");
  }else{
    ROS_INFO("Close Failed.");
  }
}

void Shoot::ShootInit() {
  armor_msgs_sub_ = nh_.subscribe<roborts_msgs::ArmorMsgs>("armors_info", 2, &Shoot::ArmorMsgsCallback, this); //BOOST_TT_HAS_MINUS_HPP_INCLUDED
  robot_shoot_sub = nh_.subscribe<roborts_msgs::RobotShoot>("robot_shoot", 2, &Shoot::RobotsShootCallBack, this);
  gimbal_cmd_pub_ = nh_.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 2);
  shoot_client = nh_.serviceClient<roborts_msgs::ShootCmd>("cmd_shoot");
  fric_client = nh_.serviceClient<roborts_msgs::FricWhl>("cmd_fric_wheel");
  ros::service::waitForService("cmd_shoot");
  ros::service::waitForService("cmd_fric_wheel");

  fricCtrl.request.open = true;
  if(fric_client.call(fricCtrl)) {
    ROS_INFO("Open Fric successfully.");
  }else{
    ROS_INFO("Open Failed.");
  }
  //shoot_thread_ = std::thread(&Shoot::ShootThread, this);
}

void Shoot::ShootThread() {
  while(ros::ok()) {
    if(shoot_num == 0) continue;
    if(enable_shoot) CallForShootService(shoot_num);
    ros::Duration(shoot_num <= 2 ? 0.5 : 1).sleep();
  }
}



//air friction is considered
float Shoot::BulletModel(float x, float v, float angle) { //x:m,v:m/s,angle:rad
  float t, y;
  t = (float)((exp(init_k_ * x) - 1) / (init_k_ * v * cos(angle)));
  y = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
  return y;
}

//x:distance , y: height
float Shoot::GetPitch(float x, float y, float v) {
  float y_temp, y_actual, dy;
  float a;
  y_temp = y;
  // by iteration
  for (int i = 0; i < 60; i++) {
    a = (float) atan2(y_temp, x);
    y_actual = BulletModel(x, v, a);
    dy = y - y_actual;
    y_temp = y_temp + dy;
    if (fabsf(dy) < 0.000001) {
      break;
    }
    //printf("iteration num %d: angle %f,temp target y:%f,err of y:%f\n",i+1,a*180/3.1415926535,yTemp,dy);
  }
  return a;
}

double Shoot::GetNextY(double x, double y, double k) {
  int nextx = (int) (x + 1.01);
  return y + (nextx - x) * k;
}


bool Shoot::CheckVerticalLineCross(int x, int y_1, int y_2) {
  int map_height = costmap_ptr_->GetCostMap()->GetSizeYCell();
  if(y_1 > y_2) std::swap(y_1, y_2);
  y_1 = std::max(0, y_1 - bullet_min_cell_dis), y_2 = std::min(y_2 + bullet_min_cell_dis, map_height - 1);
  //iterate over all high obstacles
  for(int i = 0; i < 4; i++) {
    int bl_x = grass_obstacle_info[i][0], bl_y = grass_obstacle_info[i][1], tr_x = grass_obstacle_info[i][0] + grass_obstacle_info[i][2], tr_y = grass_obstacle_info[i][1] + grass_obstacle_info[i][3];
    if(x >= bl_x && x <= tr_x && !(y_2 < bl_y || y_1 > tr_y)) return false;
  }
  return true;
}

bool Shoot::CheckCrossWithHighObstacle(geometry_msgs::PoseStamped start,geometry_msgs::PoseStamped end){
  unsigned int startx, starty, endx, endy;
  costmap_ptr_->GetCostMap()->World2Map(start.pose.position.x, start.pose.position.y, startx, starty);
  costmap_ptr_->GetCostMap()->World2Map(end.pose.position.x, end.pose.position.y, endx, endy);
  if(startx > endx) std::swap(startx, endx), std::swap(starty, endy);
  if(startx == endx) {
    if(!CheckVerticalLineCross(startx, starty, endy)) return false;
    return true;
  }else {
    double k = (double)((int)endy - (int)starty) / ((int)endx - (int)startx);
    if(!CheckVerticalLineCross(startx, starty, (int)(GetNextY(startx + 0.5, starty + 0.5, k)) + (starty < endy ? 1 : 0) )) return false;
    double prev_y =  GetNextY(startx + 0.5, starty + 0.5, k);
    for(int x_ = startx + 1; x_ < endx; x_++) {
      double next_y = GetNextY(x_, prev_y, k);
      if(!CheckVerticalLineCross(x_, (int)prev_y + (starty > endy ? 1 : 0), (int)next_y + (starty < endy ? 1 : 0))) return false;
      prev_y = next_y;
    }
    if(!CheckVerticalLineCross(endx, prev_y, endy)) return false;
    return true;
  }
}

/*
void Shoot::PublishPitchYawMsgs(const float &next_yaw, const float &next_pitch) {
  roborts_msgs::GimbalAngle gimbal_cmd_info;

  gimbal_cmd_info.yaw_mode = true;
  gimbal_cmd_info.pitch_mode = false;
  gimbal_cmd_info.yaw_angle = next_yaw;
  gimbal_cmd_info.pitch_angle = next_pitch;
  gimbal_cmd_pub_.publish(gimbal_cmd_info);
}*/

void Shoot::PublishPitchYawMsgs(const float &next_yaw, const float &next_pitch) {
    std::cout << "Pitch: " << next_pitch << " Yaw: " << next_yaw << "\n";
    roborts_msgs::GimbalAngle gimbal_cmd_info;

    float tmp_yaw = 0, total_yaw = 0;
    ros::Rate loop_rate(20);
    while(fabs(total_yaw - next_yaw) > 0.001) {
        tmp_yaw = (next_yaw < 0 ? -1 : 1) * yaw_change_unit_;
        total_yaw += tmp_yaw;
        if(next_yaw > 0) {
            if(total_yaw > next_yaw) {
                tmp_yaw -= (total_yaw - next_yaw);
                total_yaw = next_yaw;
            }
        }
        else {
            if(total_yaw < next_yaw) {
                tmp_yaw += (next_yaw - total_yaw);
                total_yaw = next_yaw;
            }
        }

        gimbal_cmd_info.yaw_mode = true;
        gimbal_cmd_info.pitch_mode = false;
        gimbal_cmd_info.yaw_angle = tmp_yaw;
        gimbal_cmd_info.pitch_angle = next_pitch;
        gimbal_cmd_pub_.publish(gimbal_cmd_info);
        loop_rate.sleep();
    }
}

void Shoot::CallForShootService(const int &shoot_num) {
  shootSrv.request.mode = 1;
  shootSrv.request.number = shoot_num;
  if(!shoot_client.call(shootSrv)) ROS_WARN ("Erros occured, can't shoot!");
  else ROS_INFO("Shoot Successfully");
}

void Shoot::ArmorMsgsCallback(const roborts_msgs::ArmorMsgs::ConstPtr &msgs) {
  cur_step = (cur_step + 1) % update_step;
  roborts_msgs::ArmorMsg choosed_armor;
  if(!ChooseArmorForShoot(msgs->detected_info, choosed_armor, shoot_num)) {
    ROS_INFO("No valid armor for shoot.");
    //if(enable_shoot) ros::Duration(1.0).sleep();
    return;
  }

  double x = choosed_armor.pose.x + offset_x, y = choosed_armor.pose.y, z = choosed_armor.pose.z;
  double dis = sqrt(x * x + y * y);
  float next_pitch = -GetPitch(dis, -h , shoot_speed_);

  float next_yaw = (float) (atan2(y, x)); //+ 5 * PI / 180.0;
  if(fabs(next_yaw) < yaw_change_threshold) next_yaw = 0;
  if(fabs(next_pitch - cur_pitch_) < pitch_change_threshold) next_pitch = cur_pitch_;

  ROS_INFO("armor_x:%.4f, armor_y:%.4f, Pitch: %.4f yaw: %.4f, shoot_num:%d", choosed_armor.pose.x, choosed_armor.pose.y, next_pitch, next_yaw, shoot_num);
  PublishPitchYawMsgs(next_yaw, next_pitch);
  //if(enable_shoot) CallForShootService(shoot_num);

  have_last_armor_ = true;
  cur_pitch_ = next_pitch, cur_yaw_ = next_yaw;
  //if(enable_shoot) ros::Duration(1.0).sleep();
}

void Shoot::RobotsShootCallBack(const roborts_msgs::RobotShoot::ConstPtr &msgs) {
  shoot_speed_ = msgs->speed;
}


 bool Shoot::GetArmorPose(geometry_msgs::Point &armor_pose_, geometry_msgs::PoseStamped &global_pose) {
  geometry_msgs::PoseStamped armor_pose;
  armor_pose.pose.position.x = armor_pose_.x;
  armor_pose.pose.position.y = armor_pose_.y;
  armor_pose.pose.orientation.w =  1.0;
  armor_pose.header.frame_id = "camera";
  armor_pose.header.stamp = ros::Time(0);

  try {
    tf_ptr_->transformPose("map", armor_pose, global_pose);
  }
  catch (tf::LookupException &ex) {
    ROS_ERROR("No Transform Error looking up robot pose: %s", ex.what());
    return false;
  }
  catch (tf::ConnectivityException &ex) {
    ROS_ERROR("Connectivity Error looking up robot pose: %s", ex.what());
    return false;
  }
  catch (tf::ExtrapolationException &ex) {
    ROS_ERROR("Extrapolation Error looking up robot pose: %s", ex.what());
    return false;
  }
  return true;
}


bool Shoot::ChooseArmorForShoot(std::vector<roborts_msgs::ArmorMsg> msgs, roborts_msgs::ArmorMsg &choosed_armor, int &shoot_num) {
  //debug
  /*double area = -1;
  for(auto armor : msgs) {
    if(armor.color == enermy_color_ && armor.area > area) {
      choosed_armor = armor;
      area = armor.area;
    }
  }
  if(area > min_area_for_resample) {
    shoot_num = 1;
    return true;
  }else return false;*/

  //debug

  //the distance between current armor position and previous armor position
  double dis  = 10000;
  geometry_msgs::PoseStamped armor_pose;
  geometry_msgs::PoseStamped min_dis_pos;

  //exist last armor for shoot
  if(have_last_armor_) {
    for(auto armor : msgs) {
      //if(armor.color != enermy_color_) continue;
      if(GetArmorPose(armor.pose, armor_pose)) {
        geometry_msgs::PoseStamped robort_pose;
        costmap_ptr_ -> GetRobotPose(robort_pose);
        ROS_INFO("have, roborts_pose: %.4f, %.4f armor_pose: %.4f, %.4f", robort_pose.pose.position.x, robort_pose.pose.position.y, armor_pose.pose.position.x, armor_pose.pose.position.y);
        if(!CheckCrossWithHighObstacle(robort_pose, armor_pose)) {
          ROS_INFO("Cross High obstacles.");
          continue;
        }
        double cur_dis = Distance(last_armor_for_shoot_, armor_pose);
        if(cur_dis < dis) {dis = cur_dis; min_dis_pos = armor_pose; choosed_armor = armor;}
      }
    }
    double dis = sqrt(choosed_armor.pose.x * choosed_armor.pose.x + choosed_armor.pose.y * choosed_armor.pose.y);
    if(dis > min_dis_for_resample || choosed_armor.area < min_area_for_resample) {
      last_armor_for_shoot_ = min_dis_pos;
      have_last_armor_ = false;
      shoot_num = 0;
      return false;
    }else{
      shoot_num = dis < 2.0 ? 5 : 2;
      return true;
    }
  }else{
    double optimal_point = -1;
    //find optimal armor
    for(auto &armor : msgs) {
      //if(armor.color != enermy_color_) continue;
      if(GetArmorPose(armor.pose, armor_pose)) {
        geometry_msgs::PoseStamped robort_pose;
        costmap_ptr_ -> GetRobotPose(robort_pose);
        ROS_INFO("dont have, roborts_pose: %.4f, %.4f armor_pose: %.4f, %.4f", robort_pose.pose.position.x, robort_pose.pose.position.y, armor_pose.pose.position.x, armor_pose.pose.position.y);
        if(!CheckCrossWithHighObstacle(robort_pose, armor_pose)) {
          ROS_INFO("Croos High obstacles.");
          continue;
        }
      }else {
        continue;
      }
      double dis = sqrt(armor.pose.x * armor.pose.x + armor.pose.y * armor.pose.y);
      //the points for this armor
      double armor_point = (5 - dis) / 5 + (double)armor.area / 2000;
      if(armor_point > optimal_point) {
        optimal_point = armor_point;
        choosed_armor = armor;
      }
    }
    //whether to choose the optimal armor
    if(optimal_point > 0 && choosed_armor.area > min_area_for_resample) {
      //if(cur_step == 0) {
        have_last_armor_ = true;
        GetArmorPose(choosed_armor.pose, last_armor_for_shoot_);
        ROS_INFO("chooose, armor_pose: %.4f, %.4f", last_armor_for_shoot_.pose.position.x, last_armor_for_shoot_.pose.position.y);
      //}
      shoot_num = 1;
      return true;
    }else{
      shoot_num = 0;
      return false;
    }
  }
}

double Shoot::Distance(geometry_msgs::PoseStamped &p_1, geometry_msgs::PoseStamped &p_2) {
  return sqrt((p_1.pose.position.x - p_2.pose.position.x) * (p_1.pose.position.x - p_2.pose.position.x) + (p_1.pose.position.y - p_2.pose.position.y) * (p_1.pose.position.y - p_2.pose.position.y));
}

//void ShootSigintHandler(int sig) {ros::shutdown();}

int main(int argc, char **argv) {
  ros::init(argc, argv, "shoot");
  Shoot *shoot = new Shoot();
  //signal(SIGINT, ShootSigintHandler);
  //ros::AsyncSpinner async_spinner(2);
  //async_spinner.start();

  ros::spin();
  ros::waitForShutdown();
}
