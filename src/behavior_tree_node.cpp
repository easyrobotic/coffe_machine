#include "coffe_machine/coffe_machine.h"
#include <iostream>
#include <fstream>
#include <chrono>

using namespace BT;

static const char* xml_text;

std::string path = "/home/julia/tfm/src/coffe_machine/xml/coffe_machine.xml";
//std::string path = "./../xml/coffe_machine.xml";
//std::string path = "./coffe_machine_v2.xml";

Blackboard::Ptr blackboard=Blackboard::create();


int main(int argc, char **argv)
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

    //ROS Node

    ros::init(argc, argv, "coffe_machine_main_node");
    ros::NodeHandle nh;
    //ros::Rate loop_rate(10);
    CoffeMachineROSNode coffe_machine_ros_node = CoffeMachineROSNode(&nh);


    /******************************************************CONDITIONS***********************************************************************/
    /**********OpenCoffeMachine Subtree*******/
    factory.registerSimpleCondition("IsMachineOpen", std::bind(&CoffeMachineROSNode::IsMachineOpen, &coffe_machine_ros_node)); 
    factory.registerSimpleCondition("IsCleanCupReady", std::bind(&CoffeMachineROSNode::IsCleanCupReady, &coffe_machine_ros_node)); 
    /**********AutoClean Subtree*******/
    factory.registerSimpleCondition("IsCleanProcessFinished", std::bind(&CoffeMachineROSNode::IsCleanProcessFinished,&coffe_machine_ros_node)); 
    factory.registerSimpleCondition("IsThereEnoughWater", std::bind(&CoffeMachineROSNode::IsThereEnoughWater,&coffe_machine_ros_node)); 
    factory.registerSimpleCondition("IsWaterTankRemoved", std::bind(&CoffeMachineROSNode::IsWaterTankRemoved, &coffe_machine_ros_node));
    factory.registerSimpleCondition("IsWaterTankFull", std::bind(&CoffeMachineROSNode::IsWaterTankFull,&coffe_machine_ros_node));   
    factory.registerSimpleCondition("IsWaterTankPlacedInCoffeMachine", std::bind(&CoffeMachineROSNode::IsWaterTankPlacedInCoffeMachine,&coffe_machine_ros_node));  
    factory.registerSimpleCondition("IsMarroTankFull", std::bind(&CoffeMachineROSNode::IsMarroTankFull,&coffe_machine_ros_node));      
    factory.registerSimpleCondition("IsMarroTankRemoved", std::bind(&CoffeMachineROSNode::IsMarroTankRemoved,&coffe_machine_ros_node));  
    factory.registerSimpleCondition("IsMarroTankEmpty", std::bind(&CoffeMachineROSNode::IsMarroTankEmpty,&coffe_machine_ros_node));     
    factory.registerSimpleCondition("IsMarroTankPlacedInCoffeMachine", std::bind(&CoffeMachineROSNode::IsMarroTankPlacedInCoffeMachine,&coffe_machine_ros_node));    
    /**********PutCoffeCup Subtree*******/
    factory.registerSimpleCondition("IsCoffeCupReady", std::bind(&CoffeMachineROSNode::IsCoffeCupReady,&coffe_machine_ros_node));     

    /******************************************************ACTIONS***********************************************************************/
    /**********OpenCoffeMachine Subtree*******/
    factory.registerSimpleAction("SwitchOnCoffeMachine", std::bind(&CoffeMachineROSNode::SwitchOnCoffeMachine, &coffe_machine_ros_node)); 
    /**********AutoClean Subtree*************/    
    factory.registerSimpleAction("FillWaterTank", std::bind(&CoffeMachineROSNode::FillWaterTank, &coffe_machine_ros_node));
    factory.registerSimpleAction("EmptyMarroTank", std::bind(&CoffeMachineROSNode::EmptyMarroTank, &coffe_machine_ros_node));   
    /**********PutCoffeCup Subtree*************/       
    factory.registerSimpleAction("PlaceCoffeCup", std::bind(&CoffeMachineROSNode::PlaceCoffeCup, &coffe_machine_ros_node));  

         
    
    
    
     

    
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



    //---------------------------------------
    // keep executin tick until it returns etiher SUCCESS or FAILURE
    // In case Asyncronous action are needed, can return Running
    
    //while (ros::ok())
    //{
    ros::Rate rate(30);
    //std::cout << "holis" << std::endl;
    while( ros::ok() && tree.tickRoot() == NodeStatus::RUNNING)
    {
        //std::this_thread::sleep_for( std::chrono::milliseconds(10) ); // crear un ros rate i fer rate.sl
        //while (ros::ok())
        //{
        
        ros::spinOnce();
        rate.sleep(); //rest time of cycle makes an sleep
        //}
    }
        //if ( tree.tickRoot() == NodeStatus::SUCCESS){
        //std::cout << tree.tickRoot() << std::endl;
        //tree.tickRoot();

        //}
        //std::cout << "holis2" << std::endl;
        //loop_rate.sleep();
        
    //}
     //tree.tickRoot(); //does in while 
    

    std::cout << "holis" << std::endl;

    return 0;

}

/* Expected output:
*
       [ Battery: OK ]
       GripperInterface::open
       ApproachObject: approach_object
       GripperInterface::close
*/