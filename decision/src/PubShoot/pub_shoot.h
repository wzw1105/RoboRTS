#include "share_head.h"
using namespace BT;
class PubShoot : public CoroActionNode {
    public:
        PubShoot(const std::string &name) : CoroActionNode(name, {})
        {
            this->_halt_request.store(false);
            this->shoot_pub_ = nh_.advertise<roborts_msgs::ShootControl>("shoot_control",1);
            this->seq = 0;
        }
        //void init(ros::NodeHandle &nh);
        NodeStatus tick() override;
        void halt() override;


    private:
        std::atomic_bool _halt_request;
        ros::NodeHandle nh_;
        ros::Publisher shoot_pub_;
        uint8_t seq;
};