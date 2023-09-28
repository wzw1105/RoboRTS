#include "share_head.h"
#include <roborts_msgs/FricWhl.h>

class SimpleActions {
  public:
      SimpleActions();
      ~SimpleActions();
      BT::NodeStatus AwakeFrichwl();
      

  private:
      ros::NodeHandle nh_;
      ros::ServiceClient Frichwl_service_client_;
};