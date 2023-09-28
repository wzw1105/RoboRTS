#ifndef _TEST_H_
#define _TEST_H_

#include "share_head.h"

namespace BT
{
// This template specialization is needed only if you want
// to AUTOMATICALLY convert a NodeParameter into a Pose2D
// In other words, implement it if you want to be able to do:
//
//   TreeNode::getInput<Pose2D>(key, ...)
//
template <> inline
Pose2D convertFromString(StringView key)
{
    // three real numbers separated by semicolons
    auto parts = BT::splitString(key, ';');
    if (parts.size() != 3)
    {
        throw BT::RuntimeError("invalid input)");
    }
    else
    {
        Pose2D output;
        output.x     = convertFromString<double>(parts[0]);
        output.y     = convertFromString<double>(parts[1]);
        output.theta = convertFromString<double>(parts[2]);
        return output;
    }
}
} // end namespace BT


class TestAsync : public BT::AsyncActionNode
{
  private:
    std::atomic_bool _halt_requested;
    double r;
  public:
    TestAsync(const std::string& name, const BT::NodeConfiguration& config)
      : AsyncActionNode(name, config){
      }
    ~TestAsync(){}
    static BT::PortsList providedPorts(){
        return{ BT::InputPort<Pose2D>("goal") };
    }


    BT::NodeStatus tick() override;

    virtual void halt() override;
};


// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<TestAsync>("TestAsync");
}

BT::NodeStatus TestAsync::tick()
{
    Pose2D goal;
    if ( !getInput<Pose2D>("goal", goal))
    {
        throw BT::RuntimeError("missing required input [goal]");
    }

    printf("[ MoveBase: STARTED ]. goal: x=%.f y=%.1f theta=%.2f\n", goal.x, goal.y, goal.theta);

    _halt_requested.store(false);
    int count = 0;

    // Pretend that "computing" takes 250 milliseconds.
    // It is up to you to check periodicall _halt_requested and interrupt
    // this tick() if it is true.
    while (!_halt_requested && count++ < 25)
    {
        SleepMS(10);
        
    }

    std::cout << "[ TestAsync: FINISHED ]" << std::endl;
    return _halt_requested ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}


void TestAsync::halt()
{
    std::cout<< "halt called! " << std::endl;
    _halt_requested.store(true);
}



#endif