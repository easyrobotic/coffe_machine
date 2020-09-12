#include <iostream>
#include "coffe_machine/coffe_machine.h"

namespace CoffeMachineNS
{

    //Global Variables
    static Global global;
    BT::NodeStatus status;

    //Global Functions


    BT::NodeStatus Global::AskForStatus(std::string NodeName)
    {

        std::cout << "Please enter an " << NodeName << " NodeStatus value: S, R or F" << std::endl;
        std::cin >> texto;

        if (texto.compare("S")==0) {
            std::cout << "The " << NodeName << " NodeStatus is Success" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        else if (texto.compare("R")==0) {
            std::cout << "The " << NodeName <<  " NodeStatus is Running" << std::endl;
            return BT::NodeStatus::RUNNING;


        }
        else{
            std::cout << "The " << NodeName << " NodeStatus is Failure" << std::endl;
            return BT::NodeStatus::FAILURE;


        }
    }


    //OpenCoffeMachine Subtree


    BT::NodeStatus IsMachineOpen()
    {
        status = global.AskForStatus("IsMachineOpen");
        return status;
        //return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus IsCleanCupReady()
    {
        status = global.AskForStatus("IsCleanCupReady");
        return status;
        //return BT::NodeStatus::SUCCESS;
    }


    BT::NodeStatus CoffeMachine::Open()
    {
        _opened = true;
        status = global.AskForStatus("SwitchOnCoffeMachine");
        return status;
        //return BT::NodeStatus::SUCCESS;
    }


    //AutoClean Subtree

     BT::NodeStatus IsCleanProcessFinished()
    {
        status = global.AskForStatus("IsCleanProcessFinished");
        return status;
        //return BT::NodeStatus::SUCCESS;

    }

       BT::NodeStatus IsThereEnoughWater()
    {
        status = global.AskForStatus("IsThereEnoughWater");
        return status;
        //return BT::NodeStatus::SUCCESS;

    }

        BT::NodeStatus IsWaterTankRemoved()
    {
        status = global.AskForStatus("IsWaterTankRemoved");
        return status;
        //return BT::NodeStatus::SUCCESS;

    }

    BT::NodeStatus IsWaterTankFull()
    {
        status = global.AskForStatus("IsWaterTankFull");
        return status;
        //std::cout << "[ IsWaterTankFull: OK ]" << std::endl;
        //return BT::NodeStatus::SUCCESS;

    }


    BT::NodeStatus WaterTank::Fill()
    {
        _filled = true;
        status = global.AskForStatus("FillWaterTank");
        return status;
        //std::cout << "[ FillingWaterTank: OK ]" << std::endl;
        //return BT::NodeStatus::SUCCESS;
    }


       BT::NodeStatus IsWaterTankPlacedInCoffeMachine()
    {
        status = global.AskForStatus("IsWaterTankPlacedInCoffeMachine");
        return status;
        //std::cout << "[ IsThereEnoughWater: OK ]" << std::endl;
        //return BT::NodeStatus::SUCCESS;

    }


       BT::NodeStatus IsMarroTankFull()
    {
        status = global.AskForStatus("IsMarroTankFull");
        return status;
        //std::cout << "[ IsMarroTankFull: OK ]" << std::endl;
        //return BT::NodeStatus::SUCCESS;

    }
       BT::NodeStatus IsMarroTankRemoved()
    {
        status = global.AskForStatus("IsMarroTankRemoved");
        return status;
        //std::cout << "[ IsMarroTankRemoved: OK ]" << std::endl;
        //return BT::NodeStatus::SUCCESS;

    }

        BT::NodeStatus IsMarroTankEmpty()
    {
        status = global.AskForStatus("IsMarroTankEmpty");
        return status;
        //std::cout << "[ IsMarroTankEmpty: OK ]" << std::endl;
        //return BT::NodeStatus::SUCCESS;

    }

       BT::NodeStatus MarroTank::Empty()
    {
        _empty = true;
        status = global.AskForStatus("EmptyMarroTank");
        return status;
        //std::cout << "[ EmptyMarroTank: OK ]" << std::endl;
        //return BT::NodeStatus::SUCCESS;
    }

        BT::NodeStatus IsMarroTankPlacedInCoffeMachine()
    {

        status = global.AskForStatus("IsMarroTankPlacedInCoffeMachine");
        return status;
        //std::cout << "[ IsMarroTankPlacedInCoffeMachine: OK ]" << std::endl;
        //return BT::NodeStatus::SUCCESS;

    }

    //PutCoffeCup Subtree


        BT::NodeStatus IsCoffeCupReady()
    {

        status = global.AskForStatus("IsCoffeCupReady");
        return status;
        //std::cout << "[ IsCoffeCupReady: OK ]" << std::endl;
        //return BT::NodeStatus::SUCCESS;

    }

        BT::NodeStatus CoffeMachine::PlaceCoffeCup()
    {
        _coffecuplaced = true;
        status = global.AskForStatus("PlaceCoffeCup");
        return status;
        //std::cout << "[ CoffeCupPlaced: OK ]" << std::endl;
        //return BT::NodeStatus::SUCCESS;
    }

    //CoffeType Subtree


        BT::NodeStatus IsDesiredCoffeSelected()
    {
        status = global.AskForStatus("IsDesiredCoffeSelected");
        return status;
        //std::cout << "[ IsDesiredCoffeSelected: OK ]" << std::endl;
        //return BT::NodeStatus::SUCCESS;

    }
        BT::NodeStatus CoffeMachine::PressDesiredCoffe()
    {
        _desiredcoffepressed = true;
        status = global.AskForStatus("PressDesiredCoffe");
        return status;
        //std::cout << "[ PressDesiredCoffe: OK ]" << std::endl;
        //return BT::NodeStatus::SUCCESS;
    }

        BT::NodeStatus IsCoffeFinished()
    {
        status = global.AskForStatus("IsCoffeFinished");
        return status;
        //std::cout << "[ IsCoffeFinished: OK ]" << std::endl;
        //return BT::NodeStatus::SUCCESS;

    }

       BT::NodeStatus HasCupOfCoffeBeenRemoved()
    {
        status = global.AskForStatus("HasCupOfCoffeBeenRemoved");
        return status;
        //std::cout << "[ HasCupOfCoffeBeenRemoved: OK ]" << std::endl;
        //return BT::NodeStatus::SUCCESS;

    }
    

       BT::NodeStatus HasHumanAddedCleaningCup()
    {
        status = global.AskForStatus("HasHumanAddedCleaningCup");
        return status;        
        //std::cout << "[ HasHumanAddedCleaningCup: OK ]" << std::endl;
        //return BT::NodeStatus::SUCCESS;

    }

        BT::NodeStatus CoffeMachine::PlaceCleanCup()
    {
        _cleancuplaced = true;
        status = global.AskForStatus("PlaceCleanCup");
        return status;  
        //std::cout << "[ PlaceCleanCup: OK ]" << std::endl;
        //return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus CoffeMachine::Close()
    {
        _opened = false;
        status = global.AskForStatus("SwitchOffCoffeMachine");
        return status; 
        //std::cout << "[ SwitchOffCoffeMachine: OK ]" << std::endl;
        //return BT::NodeStatus::SUCCESS;
    }
        BT::NodeStatus HasHumanAddedMilk()
    {
        status = global.AskForStatus("HasHumanAddedMilk");
        return status; 
        //std::cout << "[ HasHumanAddedMilk: OK ]" << std::endl;
        //return BT::NodeStatus::SUCCESS;
    }

        BT::NodeStatus IsMilkDesired()
    {
        status = global.AskForStatus("IsMilkDesired");
        return status; 
        //std::cout << "[ IsMilkDesired: OK ]" << std::endl;
        //return BT::NodeStatus::SUCCESS;
    }
    
        BT::NodeStatus IsSugarDesired()
    {
        status = global.AskForStatus("IsSugarDesired");
        return status;         
        //std::cout << "[ IsSugarDesired: OK ]" << std::endl;
        //return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus Milk::Add()
    {
        _added = false;
        status = global.AskForStatus("AddMilktoCoffe");
        return status;  
        //std::cout << "[ AddMilktoCoffe: OK ]" << std::endl;
        //return BT::NodeStatus::SUCCESS;
    }

    
    BT::NodeStatus Sugar::Add()
    {
        _added = false;
        status = global.AskForStatus("AddSugarToCoffe");
        return status;  
        //std::cout << "[ AddSugartoCoffe: OK ]" << std::endl;
        //return BT::NodeStatus::SUCCESS;
    }

    
    

} //comprovar que el xml està bé.  Ja més endavant entrarem en detall.