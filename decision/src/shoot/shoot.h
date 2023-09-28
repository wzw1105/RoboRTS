/*
 * @content: 保持安静
 * @Description: file 保持安静
 * @author: Chen
 */
/*
 * @content: 保持安静
 * @Description: file 保持安静
 * @author: Chen
 */
#include "share_head.h"
#include "enum_class.h"
using namespace BT;
using namespace ENUM_CLASS;


class Shoot : public CoroActionNode {
    public :
        Shoot(const std::string &name , const BT::NodeConfiguration& config) : 
            CoroActionNode(name, config)
        {
            this->_halt_request.store(true);
            this->shoot_cmd_msg.request.mode = this->shoot_cmd_msg.request.STOP;
            this->shoot_cmd_msg.request.number = 0;
            this->shoot_cmd_msg.response.received = false;
            this->circle = 0;
            this->shoot_control_sub_ = this->nh_.subscribe<roborts_msgs::ShootControl>("shoot_control", 2, &Shoot::Shoot_Control_CallBack_, this);
        }
        ~Shoot(){};

        static BT::PortsList providedPorts();

        void init(GimbalExecutor*& gimbal_executor);
        NodeStatus tick() override;
        void halt() override;
        void stop();
        void Shoot_Control_CallBack_(const roborts_msgs::ShootControl::ConstPtr &shoot_control);

    private:
        std::atomic_bool _halt_request;
        ros::NodeHandle nh_;
        uint8_t circle;
        ros::Subscriber shoot_control_sub_;
        GimbalExecutor *gimbal_executor_;
        roborts_msgs::ShootCmd shoot_cmd_msg;
};
