#include "ldcp/device.h"

#include <arpa/inet.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>

#define LOOP_RATE 15


int main(int argc, char *argv[]) {   
    //指定激光雷达的IP地址和端口
    //XDR0102的出厂默认IP地址为192.168.10.160,端口固定为2105
    in_addr_t ip_addr = inet_addr("192.168.10.160");
    in_port_t port = htons(2105);
    ldcp_sdk::NetworkLocation location(ip_addr, port);

    //根据设备地址信息构造Device类实例
    //激光雷达的主要功能都可通过该类来调用
    ldcp_sdk::Device device(location);

    if (device.open() != ldcp_sdk::no_error)
    {
        std ::cerr <<"Unable to open device" << std::endl;
        return -1;
    }

    ros::init(argc, argv, "roborts_laser_node");
    ros::NodeHandle n;
    ros::Publisher LaserScan_Publisher = n.advertise<sensor_msgs::LaserScan>("scan", 10);

    ros::Rate loop_rate(LOOP_RATE);
    
    // 启动雷达的数据传输
    device.startStreaming();

    while(ros::ok()) {
        sensor_msgs::LaserScan scan;
        ldcp_sdk::ScanFrame scan_frame;
        if (device.readScanFrame(scan_frame) != ldcp_sdk::no_error)
            break;
        int count = scan_frame.layers[0].ranges.size();
        for (int i = 0; i < count; i++) {
            double dis = (double)scan_frame.layers[0].ranges[i] * 0.002;
            //if(dis < 0.1) dis=10.0;
            scan.ranges.push_back(dis);
            scan.intensities.push_back(scan_frame.layers[0].intensities[i]);
            //std::cout << scan_frame.layers[0].ranges[i] * 0.002 <<" ";           
        }
        //std::cout << "\n\n";
        // std::cout << "scan ranges size: "  << scan.ranges.size() << std::endl<<std::endl;

        ros::Time scan_time = ros::Time::now();
        scan.header.seq = 1; //random
        scan.header.stamp =  scan_time;
        scan.header.frame_id =  "base_laser_link";
        scan.angle_min = -135.0 * M_PI / 180.0;
        scan.angle_max = 135.0 * M_PI / 180.0;
        scan.angle_increment = (270.0 * M_PI / 180.0) / 1536.0;
        scan.range_min = 0.1;
        scan.range_max = 10;
        scan.scan_time = 1 / LOOP_RATE;
        scan.time_increment = 1 / LOOP_RATE;

        LaserScan_Publisher.publish(scan);
        loop_rate.sleep();
    }
}