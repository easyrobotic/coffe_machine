#include <iostream>
#include "coffe_machine/coffe_machine.h"

namespace CoffeMachineNS
{
BT::NodeStatus IsMachineOpen()
{
    std::cout << "[ IsMachineOpen: OK ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus IsCleanCupReady()
{
    std::cout << "[ IsCleanCupReady: OK ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CoffeMachine::open()
{
    _opened = true;
    std::cout << "CoffeMachine::open" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CoffeMachine::close()
{
    std::cout << "CoffeMachine::close" << std::endl;
    _opened = false;
    return BT::NodeStatus::SUCCESS;
};


}
int main(){
    return 0;
}