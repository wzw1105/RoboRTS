#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <string>
#include <nav_msgs/Odometry.h>
#include "roborts_msgs/TwistAccel.h"

ros::Publisher rotation_pub;
ros::Subscriber pose_sub;
ros::Publisher cmd_vel_acc_pub_;

tf::Quaternion q_msg;
double roll, pitch, yaw, angle;
const double PI = 3.141592653589793;
const double UNIT_ANGLE = 180.0 / PI;
const double ROTATION_VEL = 1.5;

void poseCallBack(const nav_msgs::Odometry &odom) {
    tf::quaternionMsgToTF(odom.pose.pose.orientation, q_msg);
    tf::Matrix3x3(q_msg).getRPY(roll, pitch, yaw);
}

double T_angle(double a){
    a = fmod(a, 360);
    if(a > 180)a = a - 360;
    else if(a < -180)a = a + 360;
    return a;
}

void rotate_by_odom(double angle) {
    angle = T_angle(angle);
    double abs_angle;
    bool start = true;
    double init_yaw = 0;
    double rotated_angle = 0;

    geometry_msgs::Twist r_vel_msgs;
    r_vel_msgs.angular.z = angle < 0 ? -ROTATION_VEL : ROTATION_VEL;

    ros::Rate rate(100.0);
    while(ros::ok()) {
        ros::spinOnce();
        if(start) start=false, init_yaw = yaw;
        if(rotated_angle * UNIT_ANGLE < abs(angle)) {
            ros::spinOnce();
            std::cout << "yaw: " << yaw << std::endl;
            rotated_angle = abs(init_yaw - yaw);
            rotation_pub.publish(r_vel_msgs);
            //std::cout << "Angle: " << rotated_angle * UNIT_ANGLE << "\n";
        }else break;
        rate.sleep();
    }
}
void RotateByTime(double angle) {
  double freq = 25;
  angle = T_angle(angle);
  double duration = abs(angle) / UNIT_ANGLE / ROTATION_VEL;
  int ticks = int(duration * freq);
  ros::Rate rate(freq);

  std::cout << ticks << std::endl;

  roborts_msgs::TwistAccel r_vel_msgs;

  r_vel_msgs.twist.angular.z = angle < 0 ? -ROTATION_VEL : ROTATION_VEL;
  while(ros::ok()) {
    for(int i = 0; i < ticks; i++) {
      cmd_vel_acc_pub_.publish(r_vel_msgs);
      rate.sleep();
    }
    r_vel_msgs.twist.angular.z = 0;
    cmd_vel_acc_pub_.publish(r_vel_msgs);
    break;
  }
}

void RotateToPoint(tf::TransformListener &listener, tf::Point point) {
    // get the yaw of current pose;
    double current_yaw = 0;
    tf::Vector3 current_pos;
    try{
        //ros::Time now = ros::Time::now();
        tf::StampedTransform transform;
        listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/map", "/base_link",  ros::Time(0), transform);
        //tf::Quaternion qua = transform.getRotation();
        current_yaw = tf::getYaw(transform.getRotation());
        current_pos = transform.getOrigin();


        //std::cout << "pos: " << pos.getX() << " " << pos.getY() << " " << pos.getZ() << std::endl;
        //std::cout << "angle: " << tf::getYaw(qua) << "\n\n";
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        //ros::Duration(1.0).sleep();
    }

    double rotate_angle = atan2(point.getY() - current_pos.getY(), point.getX() - current_pos.getY()) - current_yaw;
    ROS_INFO("current: %lf %lf , target: %lf %lf, rotate_angle: %lf %lf", current_pos.getX(), current_pos.getY(), point.getX(), point.getY(),current_yaw, rotate_angle);
    RotateByTime(rotate_angle * UNIT_ANGLE);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_chassis");
    ros::NodeHandle n;
    cmd_vel_acc_pub_ = n.advertise<roborts_msgs::TwistAccel>("cmd_vel_acc", 100);
    tf::TransformListener listener;
    rotation_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    pose_sub = n.subscribe("/odom", 10, poseCallBack);

    double target_x, target_y;
    //std::cout << "Input degree: ";
    //std::cin >> target_x >> target_y;
    //tf::Point p(target_x, target_y, 0);
    double angle;
    std::cout << "Input degree: ";
    std::cin >> angle;

    //rotate_by_time(angle);
    RotateByTime(angle);

    //RotateToPoint(listener, p);

    /*

    
    ros::Rate rate(10.0);
    try{
        //ros::Time now = ros::Time::now();
        listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/map", "/base_link",  ros::Time(0), transform);
        tf::Quaternion qua = transform.getRotation();
        tf::Vector3 pos = transform.getOrigin();


        //std::cout << "pos: " << pos.getX() << " " << pos.getY() << " " << pos.getZ() << std::endl;
        //std::cout << "angle: " << tf::getYaw(qua) << "\n\n";
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        //ros::Duration(1.0).sleep();
    }*/

        //rate.sleep();
    //}

}