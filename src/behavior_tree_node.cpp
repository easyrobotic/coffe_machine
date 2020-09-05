#include "coffe_machine/coffe_machine.h"
#include <iostream>
#include <fstream>

using namespace BT;

static const char* xml_text;


int main()
{
    // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;

    /*std::string input_xml;
    *std::string line;
    std::ifstream in("../xml/coffe_machine.xml");
    xml_text = in;*/

    /* There are two ways to register nodes:
    *    - statically, i.e. registering all the nodes one by one.
    *    - dynamically, loading the TreeNodes from a shared library (plugin).
    * */

//#ifdef MANUAL_STATIC_LINKING
    // Note: the name used to register should be the same used in the XML.
    // Note that the same operations could be done using DummyNodes::RegisterNodes(factory)

    using namespace CoffeMachineNS;

    // The recommended way to create a Node is through inheritance.
    // Even if it requires more boilerplate, it allows you to use more functionalities
    // like ports (we will discuss this in future tutorials).
    //factory.registerNodeType<ApproachObject>("ApproachObject");

    // Registering a SimpleActionNode using a function pointer.
    // you may also use C++11 lambdas instead of std::bind
    factory.registerSimpleCondition("IsMachineOpen", std::bind(IsMachineOpen));

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
    //tree.tickRoot();

    return 0;
}

/* Expected output:
*
       [ Battery: OK ]
       GripperInterface::open
       ApproachObject: approach_object
       GripperInterface::close
*/