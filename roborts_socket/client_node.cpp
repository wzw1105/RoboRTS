#include "client_node.h"

namespace roborts_socket
{

    Client::Client()
    {

        sendBuf.num = '2';

        initrecvBuf();

        cars_pose.car_msgs.resize(4);

        //* topic publish subscribe
        posefrompost_pub_ = nh_p.advertise<roborts_msgs::CarMsgs>("pose_from_post", 2, true);
        friendpose_pub_ = nh_p.advertise<geometry_msgs::PoseStamped>("friend_pose", 2, true);
        own_pose_sub = nh_p.subscribe("amcl_pose", 2, &Client::sub_pose, this);

        start_client();
    }
    void Client::initrecvBuf()
    {

        receiveBuf.stamp_friend.tv_sec = -1;
        receiveBuf.stamp_friend.tv_nsec = -1;
        receiveBuf.stamp_guard.tv_sec = -1;
        receiveBuf.stamp_guard.tv_nsec = -1;
        receiveBuf.x = -1.0;
        receiveBuf.y = -1.0;
        for (int i = 0; i < 4; i++)
        {
            receiveBuf.robots[i].id = 0;
            receiveBuf.robots[i].color = 'o';
            // receiveBuf.robots[i].pose=0;
            // receiveBuf.robots[i].pose_num=-1;
            receiveBuf.robots[i].x = 10.0;
            receiveBuf.robots[i].y = 10.0;
        }
    }
    bool Client::build_client(int &clientSocket, sockaddr_in &addr)
    {

        // return false;
        std::cout << "build_client" << std::endl;

        // if(connect(clientSocket,(struct sockaddr*)&clientsock_in,sizeof(struct sockaddr))==0){//开始连接
        uint len = sizeof(addr);
        int sendmsg = sendto(clientSocket, (char *)&sendBuf, sizeof(MsgForSend), 0, (struct sockaddr *)&addr, len);

        if (sendmsg > 0)
        {
            std::cout << "send success" << std::endl;
        }
        int recvmsg = recvfrom(clientSocket, (char *)&receiveBuf, sizeof(PoseForRecv), 0, (struct sockaddr *)&addr, &len);
        ros::Time current_time = ros::Time::now();
    

        // std::cout<<"end_build"<<std::endl;
        if (recvmsg > 0)
        {
            ROS_INFO("stamp_guard sec: %d, nsec: %d", receiveBuf.stamp_guard.tv_sec, receiveBuf.stamp_guard.tv_nsec);

            std::cout << recvmsg << " " << current_time.sec << std::endl;
            double time_friend = current_time.sec - receiveBuf.stamp_friend.tv_sec +
                                 (current_time.nsec - receiveBuf.stamp_friend.tv_nsec) * 1e-9;
            double time_guard = current_time.sec - receiveBuf.stamp_guard.tv_sec +
                                (current_time.nsec - receiveBuf.stamp_guard.tv_nsec) * 1e-9;
            std::cout << "time friend: " << time_friend << " time guard: " << time_guard << std::endl;
            return true;
        }
        else
        {
            std::cout << recvmsg << std::endl;
            return false;
        }
    }

    void Client::start_client()
    {
        // ros::Rate loop_rate(25);

        int clientSocket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        struct sockaddr_in clientsock_in;
        memset(&clientsock_in, 0, sizeof(sockaddr_in));

        struct timeval timeout;
        timeout.tv_sec = 3;  //秒
        timeout.tv_usec = 0; //微秒
        if (setsockopt(clientSocket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1)
        {
            std::cout << "setsockopt failed:" << std::endl;
        }
        clientsock_in.sin_family = AF_INET;
        clientsock_in.sin_addr.s_addr = inet_addr("192.168.1.66");
        clientsock_in.sin_port = htons(8888);
        std::cout << "socket" << std::endl;
        while (true)
        {

            if (build_client(clientSocket_, clientsock_in))
            {

                //根据四辆车信息生成 cars_pose 并且发布
                cars_pose.stamp_guard.sec = receiveBuf.stamp_guard.tv_sec;
                cars_pose.stamp_guard.nsec = receiveBuf.stamp_guard.tv_nsec;
                // std::cout<<"time stamp:  "<< receiveBuf.stamp_guard.tv_sec <<" s,  "<<receiveBuf.stamp_guard.tv_nsec<<"ns"<<std::endl;
                roborts_msgs::CarMsg one_car_msg;
                for (int i = 0; i < 4; i++)
                {
                    one_car_msg.id = receiveBuf.robots[i].id;
                    one_car_msg.color = receiveBuf.robots[i].color;
                    // one_car_msg.pose=receiveBuf.robots[i].pose;
                    one_car_msg.x = receiveBuf.robots[i].x;
                    one_car_msg.y = receiveBuf.robots[i].y;
                    cars_pose.car_msgs[i] = one_car_msg;
                    // if(receiveBuf.robots[i].pose_num!=-1){
                    //     std::cout<<"car localization :("<< receiveBuf.robots[i].x<<" , "<<receiveBuf.robots[i].y<<")"<<std::endl;
                    // }
                }
                posefrompost_pub_.publish(cars_pose);

                //根据友方车信息生成friend_pose并发布
                friend_pose.header.stamp.sec = receiveBuf.stamp_friend.tv_sec;
                friend_pose.header.stamp.nsec = receiveBuf.stamp_friend.tv_nsec;
                friend_pose.pose.position.x = receiveBuf.x;
                friend_pose.pose.position.y = receiveBuf.y;
                friendpose_pub_.publish(friend_pose);

                // std::cout<<"pub_finish"<<std::endl;
            }
            else
            {
                std::cout << "It is blocked !" << std::endl;
            }

            //* 开启消息循环，spinOnce
            ros::spinOnce();

            //    loop_rate.sleep();//looprate
        }
        close(clientSocket_);
        return;
    }

    void Client::sub_pose(const geometry_msgs::PoseStamped::ConstPtr &own_pose)
    {
        sendBuf.stamp_myself.tv_sec = own_pose->header.stamp.sec;
        sendBuf.stamp_myself.tv_nsec = own_pose->header.stamp.nsec;
        sendBuf.x = own_pose->pose.position.x;
        sendBuf.y = own_pose->pose.position.y;
        // std::cout << "sub pose x: "<< sendBuf.x<< "y: "<< sendBuf.y << "time stamp:" << own_pose->header.stamp << std::endl;
        return;
    }

} // namespace roborts_socket end

int main(int argc, char **argv)
{
    ros::init(argc, argv, "client_node");

    roborts_socket::Client car_client_1;

    // std::thread socket_client(car_client_1.start_client);
    // socket_client.join();

    return 0;
}
