#include <iostream>
#include <ros/ros.h>
#include "roborts_msgs/GameZoneArray.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_game_zone_pub");
    ros::NodeHandle n;
    ros::Publisher game_zone_pub = n.advertise<roborts_msgs::GameZoneArray>("game_zone_array_status", 1000);

    while(ros::ok()) {
        std::cout << "input order: ";
        roborts_msgs::GameZoneArray pub_;
        for(int i = 0; i < 6; i++) std::cin >> pub_.zone[i].type;

    /*
        pub_.zone[0].type = 2;
        pub_.zone[1].type = 1;
        pub_.zone[2].type = 4;
        pub_.zone[3].type = 6;
        pub_.zone[4].type = 3;
        pub_.zone[5].type = 5;
*/
        pub_.zone[0].active = true;
        pub_.zone[1].active = true;
        pub_.zone[2].active = true;
        pub_.zone[3].active = true;
        pub_.zone[4].active = true;
        pub_.zone[5].active = true;
        game_zone_pub.publish(pub_);

    }
    
}