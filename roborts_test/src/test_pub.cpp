#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    //1000 为queue_size大小，设置为１：实时性，只保留最新的。０：表示为无穷大。
    //关于queue_size的详解：https://blog.csdn.net/handsome_for_kill/article/details/81984428
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(10);
 
    while (ros::ok())
    {
        int order;
        std::cout << "Input order: ";
        std::cin >> order;
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world ";
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        
        // 向 Topic: chatter 发送消息, 发送频率为10Hz（1秒发10次）；消息池最大容量1000。
        chatter_pub.publish(msg);
        loop_rate.sleep();
    }
    return 0;
}