
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <iostream>
namespace CoffeMachineNS{

BT::NodeStatus IsMachineOpen();
BT::NodeStatus IsCleanCupReady();

class CoffeMachine
{
    public:
        CoffeMachine() : _opened(true)
        {
        }

        BT::NodeStatus open();

        BT::NodeStatus close();

    private:
        bool _opened;
};


}   
