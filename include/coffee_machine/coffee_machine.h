
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <openpose_ros_msgs/OpenPoseHumanList.h>
#include <openpose_ros_msgs/PointWithProb.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <coffee_machine/State.h>    
#include <opencv2/highgui/highgui.hpp>
#include<unistd.h>
//#include <unistd.h>
//#include <stdio.h>
#include <dirent.h>

namespace CoffeMachineNS{

//OpenCoffeMachine Subtree

BT::NodeStatus IsMachineOpen(); //
//BT::NodeStatus IsCleanCupReady(); //

//AutoClean Subtree
BT::NodeStatus IsCupOffCoffeRemoved(); //
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


class CoffeMachineROSNode{
    public:
        CoffeMachineROSNode(ros::NodeHandle *nh);
        sensor_msgs::Image cm_raw_image;
        darknet_ros_msgs::BoundingBoxes cm_bounding_boxes;
        openpose_ros_msgs::OpenPoseHumanList cm_openpose_hl;


        /******************************************************CONDITIONS***********************************************************************/
        /**********OpenCoffeMachine Subtree*******/
        BT::NodeStatus IsMachineOpen();
        BT::NodeStatus IsCleanCupReady();
        /**********AutoClean Subtree*******/
        BT::NodeStatus IsCleanProcessFinished();
        BT::NodeStatus IsThereEnoughWater();
        BT::NodeStatus IsWaterTankRemoved();
        BT::NodeStatus IsWaterTankFull();
        BT::NodeStatus IsWaterTankPlacedInCoffeMachine();
        BT::NodeStatus IsMarroTankFull();
        BT::NodeStatus IsMarroTankRemoved();
        BT::NodeStatus IsMarroTankEmpty();
        BT::NodeStatus IsMarroTankPlacedInCoffeMachine();
        /**********PutCoffeCup Subtree*******/
        BT::NodeStatus IsCoffeCupReady();
        /**********CoffeType Subtree*******/          
        BT::NodeStatus IsDesiredCoffeSelected();
        BT::NodeStatus IsCoffeFinished();
        BT::NodeStatus HasCupOfCoffeBeenRemoved();
        BT::NodeStatus HasHumanAddedCleaningCup();
        BT::NodeStatus HasHumanAddedMilk();
        BT::NodeStatus HasHumanAddedSugar();
        BT::NodeStatus IsMilkDesired();
        BT::NodeStatus IsSugarDesired();

        /******************************************************ACTIONS***********************************************************************/
        /**********OpenCoffeMachine Subtree*******/
        BT::NodeStatus SwitchOnCoffeMachine();
        /**********AutoClean Subtree*************/ 
        BT::NodeStatus FillWaterTank();
        BT::NodeStatus EmptyMarroTank();
        /**********PutCoffeCUp Subtree*************/ 
        BT::NodeStatus PlaceCoffeCup();
        /**********CoffeType Subtree*******/        
        BT::NodeStatus PressDesiredCoffe();
        BT::NodeStatus PlaceCleanCup();
        BT::NodeStatus SwitchOffCoffeMachine();
        BT::NodeStatus AddMilkToCoffe();
        BT::NodeStatus AddSugarToCoffe();

    private:
        //ros::Publisher pub;
        ros::NodeHandle nh;
        //callback_functions
        void raw_image_callback(const sensor_msgs::Image& msg);
        void bounding_boxes_callback(const darknet_ros_msgs::BoundingBoxes& msg);
        void openpose_hl_callback(const openpose_ros_msgs::OpenPoseHumanList& msg);

        //subscriber and publisher
        ros::Subscriber raw_image_sub;
        ros::Subscriber bounding_boxes_sub;
        ros::Subscriber openpose_sub;
        image_transport::Publisher image_pub_bb_image_out;
        ros::Publisher pub_yolo_image_raw;
        ros::Publisher pub_openpose_image_raw;
        ros::Publisher bh_tree;

        // member methods as well:
        void initializeSubscribers();        
        void initializePublishers();
        void copyImage(const sensor_msgs::ImageConstPtr& msg);
        void plotBoundingBoxesInImage(int x_min, int y_min, int x_max, int y_max, int id, cv_bridge::CvImagePtr image);
        void plotJointInImage(int x, int y, int joint,cv_bridge::CvImagePtr image);
        void publishCmRawImage();
        void publishBBImage(std::string image_name,cv_bridge::CvImagePtr image);
        void publishBTState(std::string NodeType, std::string NodeStatus);

        //using to image threatment
        std::string img_encoding_;
        cv_bridge::CvImagePtr cv_img_ptr_in_;
        //cv_bridge::CvImage cv_img_out_;
        

        //internal functions
        void pub_cm_raw_image_fun();
        openpose_ros_msgs::PointWithProb find_joint(const openpose_ros_msgs::OpenPoseHumanList& msg, int num_joint);
        darknet_ros_msgs::BoundingBox find_class(const darknet_ros_msgs::BoundingBoxes& msg, std::string dk_class);

        //internal properties
        bool yolo_fk;
        bool openpose_fk;
        bool image_sent;


            //ros::ServiceServer reset_service;
};

class CoffeMachine
{
    public:
        CoffeMachine() : _opened(false)
        {
            //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
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
