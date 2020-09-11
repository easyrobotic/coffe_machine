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


    BT::NodeStatus CoffeMachine::Open()
    {
        _opened = true;
        std::cout << "CoffeMachine::open" << std::endl;
        std::string texto;
        std::cin >> texto;
        if (texto.compare("S")) {
            //0: if they are equal 
            return BT::NodeStatus::SUCCESS;
            //!0: same difference
        }

        else if (texto.compare("R")) {
            //0: if they are equal 
            return BT::NodeStatus::RUNNING;
            //!0: same difference


        }
        else{
            //0: if they are equal 
            return BT::NodeStatus::FAILURE;
            //!0: same difference


        }
        return BT::NodeStatus::SUCCESS;
    }

     BT::NodeStatus IsCleanProcessFinished()
    {
        std::cout << "[ IsCleanProcessFinished: OK ]" << std::endl;
        return BT::NodeStatus::SUCCESS;

    }

       BT::NodeStatus IsThereEnoughWater()
    {
        std::cout << "[ IsThereEnoughWater: OK ]" << std::endl;
        return BT::NodeStatus::SUCCESS;

    }

        BT::NodeStatus IsWaterTankRemoved()
    {
        std::cout << "[ IsWaterTankRemoved: OK ]" << std::endl;
        return BT::NodeStatus::SUCCESS;

    }

    BT::NodeStatus IsWaterTankFull()
    {
        std::cout << "[ IsWaterTankFull: OK ]" << std::endl;
        return BT::NodeStatus::SUCCESS;

    }


    BT::NodeStatus WaterTank::Fill()
    {
        _filled = true;
        std::cout << "[ FillingWaterTank: OK ]" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }


       BT::NodeStatus IsWaterTankPlacedInCoffeMachine()
    {
        std::cout << "[ IsThereEnoughWater: OK ]" << std::endl;
        return BT::NodeStatus::SUCCESS;

    }

       BT::NodeStatus IsMarroTankFull()
    {
        std::cout << "[ IsMarroTankFull: OK ]" << std::endl;
        return BT::NodeStatus::SUCCESS;

    }
       BT::NodeStatus IsMarroTankRemoved()
    {
        std::cout << "[ IsMarroTankRemoved: OK ]" << std::endl;
        return BT::NodeStatus::SUCCESS;

    }

        BT::NodeStatus IsMarroTankEmpty()
    {
        std::cout << "[ IsMarroTankEmpty: OK ]" << std::endl;
        return BT::NodeStatus::SUCCESS;

    }

       BT::NodeStatus MarroTank::Empty()
    {
        _empty = true;
        std::cout << "[ EmptyMarroTank: OK ]" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

        BT::NodeStatus IsMarroTankPlacedInCoffeMachine()
    {
        std::cout << "[ IsMarroTankPlacedInCoffeMachine: OK ]" << std::endl;
        return BT::NodeStatus::SUCCESS;

    }

        BT::NodeStatus IsCoffeCupReady()
    {
        std::cout << "[ IsCoffeCupReady: OK ]" << std::endl;
        return BT::NodeStatus::SUCCESS;

    }

        BT::NodeStatus CoffeMachine::PlaceCoffeCup()
    {
        _coffecuplaced = true;
        std::cout << "[ CoffeCupPlaced: OK ]" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

        BT::NodeStatus IsDesiredCoffeSelected()
    {
        std::cout << "[ IsDesiredCoffeSelected: OK ]" << std::endl;
        return BT::NodeStatus::SUCCESS;

    }
        BT::NodeStatus CoffeMachine::PressDesiredCoffe()
    {
        _desiredcoffepressed = true;
        std::cout << "[ PressDesiredCoffe: OK ]" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

        BT::NodeStatus IsCoffeFinished()
    {
        std::cout << "[ IsCoffeFinished: OK ]" << std::endl;
        return BT::NodeStatus::SUCCESS;

    }

       BT::NodeStatus HasCupOfCoffeBeenRemoved()
    {
        std::cout << "[ HasCupOfCoffeBeenRemoved: OK ]" << std::endl;
        return BT::NodeStatus::SUCCESS;

    }
    

       BT::NodeStatus HasHumanAddedCleaningCup()
    {
        std::cout << "[ HasHumanAddedCleaningCup: OK ]" << std::endl;
        return BT::NodeStatus::SUCCESS;

    }

        BT::NodeStatus CoffeMachine::PlaceCleanCup()
    {
        _cleancuplaced = true;
        std::cout << "[ PlaceCleanCup: OK ]" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus CoffeMachine::Close()
    {
        _opened = false;
        std::cout << "[ SwitchOffCoffeMachine: OK ]" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
        BT::NodeStatus HasHumanAddedMilk()
    {
        std::cout << "[ HasHumanAddedMilk: OK ]" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

        BT::NodeStatus IsMilkDesired()
    {
        std::cout << "[ IsMilkDesired: OK ]" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    
        BT::NodeStatus IsSugarDesired()
    {
        std::cout << "[ IsSugarDesired: OK ]" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus Milk::Add()
    {
        _added = false;
        std::cout << "[ AddMilktoCoffe: OK ]" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    
    BT::NodeStatus Sugar::Add()
    {
        _added = false;
        std::cout << "[ AddSugartoCoffe: OK ]" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    
    

} //comprovar que el xml està bé.  Ja més endavant entrarem en detall.