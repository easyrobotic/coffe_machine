#include "coffe_machine/coffe_machine.h"
#include <iostream>
#include <fstream>

using namespace BT;

static const char* xml_text;

//std::string path = "./home/julia/tfm/src/coffe_machine/xml/coffe_machine.xml";
std::string path = "./../xml/coffe_machine.xml";
//std::string path = "./coffe_machine_v2.xml";

Blackboard::Ptr blackboard=Blackboard::create();


int main()
{
    // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;

    using namespace CoffeMachineNS;


    // Registering a SimpleActionNode using a function pointer.
    // you may also use C++11 lambdas instead of std::bind
    static CoffeMachine coffe_machine;
    static WaterTank water_tank;
    static MarroTank marro_tank;
    static Milk milk;
    static Sugar sugar;

    factory.registerSimpleCondition("IsMachineOpen", std::bind(IsMachineOpen));
    factory.registerSimpleCondition("IsCleanCupReady", std::bind(IsCleanCupReady));
    factory.registerSimpleAction("SwitchOnCoffeMachine", std::bind(&CoffeMachine::Open, &coffe_machine));
    factory.registerSimpleCondition("IsCleanProcessFinished", std::bind(IsCleanProcessFinished));
    factory.registerSimpleCondition("IsThereEnoughWater", std::bind(IsThereEnoughWater));
    factory.registerSimpleCondition("IsWaterTankRemoved", std::bind(IsWaterTankRemoved));
    factory.registerSimpleCondition("IsWaterTankFull", std::bind(IsWaterTankFull));   
    factory.registerSimpleAction("FillWaterTank", std::bind(&WaterTank::Fill, &water_tank));
    factory.registerSimpleCondition("IsWaterTankPlacedInCoffeMachine", std::bind(IsWaterTankPlacedInCoffeMachine));       
    factory.registerSimpleCondition("IsMarroTankFull", std::bind(IsMarroTankFull));     
    factory.registerSimpleCondition("IsMarroTankRemoved", std::bind(IsMarroTankRemoved));        
    factory.registerSimpleCondition("IsMarroTankEmpty", std::bind(IsMarroTankEmpty));      
    factory.registerSimpleAction("EmptyMarroTank", std::bind(&MarroTank::Empty, &marro_tank));   
    factory.registerSimpleCondition("IsMarroTankPlacedInCoffeMachine", std::bind(IsMarroTankPlacedInCoffeMachine));     
    factory.registerSimpleCondition("IsCoffeCupReady", std::bind(IsCoffeCupReady));     
    factory.registerSimpleAction("PlaceCoffeCup", std::bind(&CoffeMachine::PlaceCoffeCup, &coffe_machine));  
    factory.registerSimpleCondition("IsDesiredCoffeSelected", std::bind(IsDesiredCoffeSelected));      
    factory.registerSimpleAction("PressDesiredCoffe", std::bind(&CoffeMachine::PressDesiredCoffe, &coffe_machine)); 
    factory.registerSimpleCondition("IsCoffeFinished", std::bind(IsCoffeFinished));     
    factory.registerSimpleCondition("HasCupOfCoffeBeenRemoved", std::bind(HasCupOfCoffeBeenRemoved));     
    factory.registerSimpleCondition("HasHumanAddedCleaningCup", std::bind(HasHumanAddedCleaningCup));    
    factory.registerSimpleAction("PlaceCleanCup", std::bind(&CoffeMachine::PlaceCleanCup, &coffe_machine)); 
    factory.registerSimpleAction("SwitchOffCoffeMachine", std::bind(&CoffeMachine::Close, &coffe_machine)); 
    factory.registerSimpleCondition("HasHumanAddedMilk", std::bind(HasHumanAddedMilk));        
    factory.registerSimpleCondition("IsMilkDesired", std::bind(IsMilkDesired));    
    factory.registerSimpleCondition("IsSugarDesired", std::bind(IsSugarDesired));   
    factory.registerSimpleAction("AddMilkToCoffe", std::bind(&Milk::Add, &milk)); 
    factory.registerSimpleAction("AddSugarToCoffe", std::bind(&Sugar::Add, &sugar));  

    std::cout << "creating tree from file" << std::endl;
    auto tree = factory.createTreeFromFile(path);
    std::cout << "tree from file is created" << std::endl;
    //You can also create SimpleActionNodes using methods of a class
    /*GripperInterface gripper;
    factory.registerSimpleAction("OpenGripper", std::bind(&GripperInterface::open, &gripper));
    factory.registerSimpleAction("CloseGripper", std::bind(&GripperInterface::close, &gripper));*/

//else
    // Load dynamically a plugin and register the TreeNodes it contains
    // it automated the registering step.
    //factory.registerFromPlugin("./libdummy_nodes_dyn.so");
//#endif

    // Trees are created at deployment-time (i.e. at run-time, but only once at the beginning).
    // The currently supported format is XML.
    // IMPORTANT: when the object "tree" goes out of scope, all the TreeNodes are destroyed

    //auto tree = factory.createTreeFromText(xml_text);
    //auto tree = factory.createTreeFromText(xml_test);

    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.
    tree.tickRoot();

    return 0;
}

/* Expected output:
*
       [ Battery: OK ]
       GripperInterface::open
       ApproachObject: approach_object
       GripperInterface::close
*/