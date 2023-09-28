#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <sensor_msgs/LaserScan.h>

void laserscanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
    std::vector<float> ranges_ = msg->ranges;
    for(int i = 0 ; i < ranges_.size(); i++) if(ranges_[i] < 0.4) std::cout << i << " " << ranges_[i] << "\n";
    std::cout << "\n\n";
}
  
int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("scan", 2, laserscanCallback);
  
    ros::Rate loop_rate(25); //频率为2５hz
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep(); //配合执行频率，sleep一段时间，然后进入下一个循环。
    }
    return 0;
}
