#include <stdio.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <ctime>
#include <thread>
#include <sys/ioctl.h>
#include <iostream>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include "roborts_msgs/CarMsgs.h"
#include "roborts_msgs/CarMsg.h"


namespace roborts_socket{

struct MsgForSend{
    timespec stamp_myself;
    char num;
    double x,y;
};
//未检测到的话id、color等值为0  
struct Robot{
    uint8_t id;
    char color;  
    double x,y;
};

struct PoseForRecv{
    timespec stamp_guard;
    timespec stamp_friend;
    double x,y;
    Robot robots[4];
    void operator=(const PoseForRecv & pose){
        stamp_guard=pose.stamp_guard;
        stamp_friend=pose.stamp_friend;
        x=pose.x;
        y=pose.y;
        memcpy(robots,pose.robots,sizeof(robots));

    }
    void setRobots(Robot * robots_){
        memcpy(robots,robots_,sizeof(robots));
    }
};

class Client{
public:
    Client();
    bool build_client(int & clientSocket, sockaddr_in & addr);
    void start_client();
    void sub_pose(const geometry_msgs::PoseStamped::ConstPtr& own_pose);
    void initrecvBuf();

private:
    //std::mutex mutex_c;
    geometry_msgs::PoseStamped friend_pose;
    roborts_msgs::CarMsgs cars_pose;
    std::vector<roborts_msgs::CarMsg> car_msgs;

    //ROS Node handle
    ros::NodeHandle nh_p;
    //ROS Publisher
    ros::Publisher posefrompost_pub_;
    ros::Publisher friendpose_pub_;
    ros::Subscriber own_pose_sub;
    MsgForSend sendBuf;
    PoseForRecv receiveBuf;
    

}; 
}//namespace
