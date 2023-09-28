/*#include "ros/ros.h"
#include "std_msgs/String.h"
  
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("In callback!!!");
}
  
int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("chatter", 2, chatterCallback);
  
    ros::Rate loop_rate(5); //频率为５hz
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep(); //配合执行频率，sleep一段时间，然后进入下一个循环。
        ROS_INFO("After spinOnce!!!");
    }
    return 0;
}
*/

#include <ros/ros.h>
 
int main(int argc, char* argv[])
{
    int self_color = -1;
    ros::init(argc, argv, "listener");
 
    //n is in the global namespace
    ros::NodeHandle n;
    n.getParam("self_color", self_color);
 
    ros::Rate loop_rate(10);
    ROS_INFO("Serial was %d", self_color);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}