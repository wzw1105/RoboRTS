/*
 * @Author: kidding 2867634589@qq.com
 * @Date: 2022-05-03 15:22:23
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2022-05-21 16:10:16
 * @FilePath: /roborts_ws/src/RoboRTS/decision/src/awaken/awaken.h
 * @Description: 这是默认设置,请设置`customMade`, 
 */
#include "share_head.h"
#include <roborts_msgs/FricWhl.h>

using namespace BT;

class Awaken : public SyncActionNode {
  public:
    Awaken(const std::string &name)
        : BT::SyncActionNode(name, {})
    {
      FricWhl_service_client_ = nh.serviceClient<roborts_msgs::FricWhl>("cmd_fric_wheel");
      this->awake = false;
    };

    ~Awaken();

    BT::NodeStatus tick() override;

    void init(GimbalExecutor *&gimbal_executor);

  private:
    ros::NodeHandle nh;
    ros::ServiceClient FricWhl_service_client_;
    
    GimbalExecutor *gimbal_executor_;
    bool awake;
};