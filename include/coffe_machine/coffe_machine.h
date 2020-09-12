
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <iostream>
namespace CoffeMachineNS{

//OpenCoffeMachine Subtree

BT::NodeStatus IsMachineOpen(); //
BT::NodeStatus IsCleanCupReady(); //

//AutoClean Subtree
BT::NodeStatus IsCleanProcessFinished(); //
BT::NodeStatus IsCupOffCoffeRemoved(); //
BT::NodeStatus IsThereEnoughWater(); //
BT::NodeStatus IsWaterTankRemoved(); //
BT::NodeStatus IsWaterTankFull(); // 
BT::NodeStatus IsWaterTankPlacedInCoffeMachine(); //
BT::NodeStatus IsMarroTankFull(); //
BT::NodeStatus IsMarroTankRemoved(); //
BT::NodeStatus IsMarroTankEmpty(); //
BT::NodeStatus IsMarroTankPlacedInCoffeMachine(); //


//PutCoffeCup Subtree

BT::NodeStatus IsCoffeCupReady(); //


//CoffeType Subtree
BT::NodeStatus IsDesiredCoffeSelected(); //
BT::NodeStatus IsCoffeFinished(); //
BT::NodeStatus HasCupOfCoffeBeenRemoved();
BT::NodeStatus HasHumanAddedCleaningCup(); //
BT::NodeStatus HasHumanAddedMilk(); //
BT::NodeStatus IsMilkDesired(); //
BT::NodeStatus IsSugarDesired(); //



//BT::NodeStatus IsCoffeMachineSwitchedOff();















class CoffeMachine
{
    public:
        CoffeMachine() : _opened(false)
        {

        }

        BT::NodeStatus Open();
        BT::NodeStatus PlaceCoffeCup();
        BT::NodeStatus PressDesiredCoffe();
        BT::NodeStatus PlaceCleanCup(); 
        BT::NodeStatus Close();       

    private:
        bool _opened;
        bool _coffecuplaced;
        bool _desiredcoffepressed;
        bool _cleancuplaced;       
};


class WaterTank
{
    public:
        WaterTank() : _filled(true)
        {

        }

        BT::NodeStatus Fill();

    private:
        bool _filled;
};

class MarroTank
{
    public:
        MarroTank() : _empty(true)
        {

        }

        BT::NodeStatus Empty();

    private:
        bool _empty;
};

class Milk
{
    public:
        Milk() : _added(true)
        {

        }

        BT::NodeStatus Add();

    private:
        bool _added;
};

class Sugar
{
    public:
        Sugar() : _added(true)
        {

        }

        BT::NodeStatus Add();

    private:
        bool _added;
};

class Global
{
    public:
        BT::NodeStatus AskForStatus(std::string NodeName);
        std::string texto;

};



/*class CleanProcess
{
    public:
        CleanProcess() : _started(true)
        {

        }

        BT::NodeStatus Start();
        BT::NodeStatus Finish();

    private:
        bool _started;
};*/



}   
