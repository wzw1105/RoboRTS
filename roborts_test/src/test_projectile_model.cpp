#include <iostream>
#include <ros/ros.h>
#include <signal.h>
#include "roborts_msgs/GimbalAngle.h"
#include "roborts_msgs/ArmorMsgs.h"
#include "roborts_msgs/ShootCmd.h"
#include "roborts_msgs/RobotShoot.h"
#include "roborts_msgs/FricWhl.h"

const float init_k_ = 0.026;
const float GRAVITY = 9.78;
const double PI = 3.1415926535;

int cnt = 0;
float shoot_speed_ = 15;

ros::Publisher gimbal_cmd_pub;
ros::Subscriber armor_info_sub;
ros::ServiceClient fric_client;
ros::ServiceClient shoot_client;
ros::Subscriber robot_shoot_sub;

//air friction is considered
float BulletModel(float x, float v, float angle) { //x:m,v:m/s,angle:rad
  float t, y;
  t = (float)((exp(init_k_ * x) - 1) / (init_k_ * v * cos(angle)));
  y = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
  return y;
}

//x:distance , y: height
float GetPitch(float x, float y, float v) {
  float y_temp, y_actual, dy;
  float a;
  y_temp = y;
  // by iteration
  for (int i = 0; i < 40; i++) {
    a = (float) atan2(y_temp, x);
    y_actual = BulletModel(x, v, a);
    dy = y - y_actual;
    y_temp = y_temp + dy;
    if (fabsf(dy) < 0.00001) {
      break;
    }
    //printf("iteration num %d: angle %f,temp target y:%f,err of y:%f\n",i+1,a*180/3.1415926535,yTemp,dy);
  }
  return a;

}

void ArmorMsgsCallback(roborts_msgs::ArmorMsgs msgs) {
  if(!msgs.detected) return;

  double x = msgs.detected_info[0].pose.x, y = msgs.detected_info[0].pose.y, z = msgs.detected_info[0].pose.z;
  double dis=sqrt(x * x+y * y);
  float pitch = -GetPitch(dis, -0.05 , shoot_speed_);

  double yaw = (float) (atan2(y, x)) + 5 * PI / 180.0;
  ROS_INFO("Pitch: %.4f yaw: %.4f",pitch, yaw);

  roborts_msgs::GimbalAngle gimbal_cmd_info;
  gimbal_cmd_info.yaw_mode=true;
  gimbal_cmd_info.pitch_mode=false;
  gimbal_cmd_info.yaw_angle=yaw;
  gimbal_cmd_info.pitch_angle=pitch;

  gimbal_cmd_pub.publish(gimbal_cmd_info);
  cnt = (cnt + 1) % 2; 
  if(cnt ==0) {
    roborts_msgs::ShootCmd shootSrv;
    shootSrv.request.mode = 1;
    shootSrv.request.number = 2;
    if(shoot_client.call(shootSrv)){
        ROS_INFO("Shoot successfully!");
    }else{
        ROS_ERROR("can't shoot!");
    }
  }
}

void RobotsShootCallBack(roborts_msgs::RobotShoot msgs) {
  ROS_INFO("speed: %.4f", shoot_speed_);
  shoot_speed_ = msgs.speed;
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "test_projectile_model");
    ros::NodeHandle n;

    gimbal_cmd_pub = n.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 2);

    robot_shoot_sub = n.subscribe("robot_shoot", 2, RobotsShootCallBack);
    shoot_client = n.serviceClient<roborts_msgs::ShootCmd>("cmd_shoot");
    fric_client = n.serviceClient<roborts_msgs::FricWhl>("cmd_fric_wheel");
    ros::service::waitForService("cmd_shoot");
    ros::service::waitForService("cmd_fric_wheel");

    roborts_msgs::FricWhl fricCtrl;
    fricCtrl.request.open = true;
    if(fric_client.call(fricCtrl)) {
      ROS_INFO("Open Fric successfully.");
    }else{
      ROS_INFO("Open Failed.");
    }

    int opt;
    std::cout << "Option: 1.input distance 2,armors_msgs callback.\n";
    std::cout << "input your choice:\n";
    std::cin >> opt;
    if(opt == 1) {
      while(true) {
        std::cout << "input distance: ";
        double dis;
        std::cin >> dis;
        if(dis < 0) {
          roborts_msgs::FricWhl fricCtrl;
          fricCtrl.request.open = false;
          if(fric_client.call(fricCtrl)) {
            ROS_INFO("Close Fric successfully.");
          }else{
            ROS_INFO("Close Failed.");
          }
          break;
        }
        float pitch = -GetPitch(dis, -0.2 , shoot_speed_);

        double yaw = 0;
        ROS_INFO("Pitch: %.4f yaw: %.4f",pitch, yaw);

        roborts_msgs::GimbalAngle gimbal_cmd_info;
        gimbal_cmd_info.yaw_mode=true;
        gimbal_cmd_info.pitch_mode=false;
        gimbal_cmd_info.yaw_angle=yaw;
        gimbal_cmd_info.pitch_angle=pitch;

        gimbal_cmd_pub.publish(gimbal_cmd_info);

        roborts_msgs::ShootCmd shootSrv;
        shootSrv.request.mode = 1;
        shootSrv.request.number = 2;
        if(shoot_client.call(shootSrv)){
            ROS_INFO("Shoot successfully!");
            ros::spinOnce();
        }else{
            ROS_ERROR("can't shoot!");
        }
      }
    }else{
      armor_info_sub = n.subscribe<roborts_msgs::ArmorMsgs>("armors_info", 2, ArmorMsgsCallback);
      ros::Rate rate_(2);
      while(ros::ok()) {
        ros::spinOnce();
        rate_.sleep();
      }
    }
  }
