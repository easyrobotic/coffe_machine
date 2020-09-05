
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <iostream>
namespace CoffeMachineNS{

BT::NodeStatus IsCupOffCoffeRemoved(); //
BT::NodeStatus IsCleanCupReady(); //
BT::NodeStatus IsCleanProcessFinished(); //
BT::NodeStatus IsCoffeCupReady(); //
BT::NodeStatus IsCoffeFinished(); //
BT::NodeStatus IsCoffeMachineSwitchedOff();
BT::NodeStatus IsDesiredCoffeSelected(); //
BT::NodeStatus HasHumanAddedCleaningCup(); //
BT::NodeStatus HasHumanAddedMilk();
BT::NodeStatus IsMachineOpen(); //
BT::NodeStatus IsMarroTankEmpty(); //
BT::NodeStatus IsMarroTankFull(); //
BT::NodeStatus IsMarroTankPlacedInCoffeMachine(); //
BT::NodeStatus IsMarroTankRemoved(); //
BT::NodeStatus IsMilkDesired(); //
BT::NodeStatus IsSugarDesired(); //
BT::NodeStatus IsThereEnoughWater(); //
BT::NodeStatus IsWaterTankFull(); //
BT::NodeStatus IsWaterTankPlacedInCoffeMachine(); //
BT::NodeStatus IsWaterTankRemoved(); //


class SwitchOnCoffeMachine
{
    public:
        SwitchOnCoffeMachine() : _opened(true)
        {
        }

        BT::NodeStatus open();


    private:
        bool _opened;
};


}   
