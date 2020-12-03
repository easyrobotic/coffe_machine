#include <iostream>
#include "coffe_machine/coffe_machine.h"


namespace CoffeMachineNS
{

    //Global Variables
    static Global global;
    BT::NodeStatus status;
    //Global Functions

    CoffeMachineROSNode::CoffeMachineROSNode(ros::NodeHandle* nodehandle)
    {
        ROS_INFO("in class constructor of ExampleRosClass");
        initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
        initializePublishers();
    }

    void CoffeMachineROSNode::initializeSubscribers()
    {
        ROS_INFO("Initializing Subscribers");
        //raw_image_sub = nh.subscribe("/usb_cam/image_raw",100,&CoffeMachineROSNode::raw_image_callback,this);
        bounding_boxes_sub = nh.subscribe("/darknet_ros/bounding_boxes",100,&CoffeMachineROSNode::bounding_boxes_callback,this);
        openpose_sub = nh.subscribe("/openpose_ros/human_list",100,&CoffeMachineROSNode::openpose_hl_callback,this);
    }

    void CoffeMachineROSNode::initializePublishers()
    {
        ROS_INFO("Initializing Publishers");
        pub_cm_raw_image = nh.advertise<sensor_msgs::Image>("/coffe_machine/image_raw", 100);  
        image_transport::ImageTransport img_tp_(nh); 
        image_pub_bb_image_out = img_tp_.advertise("/coffe_machine/image_with_output_data", 100);
    }
  

    void CoffeMachineROSNode::raw_image_callback(const sensor_msgs::Image& msg)
    {
        //std::cout << "camera_info" << msg << std::endl;
        cm_raw_image = msg;
        //std::cout << cm_raw_image << std::endl;
        pub_cm_raw_image.publish(cm_raw_image);
    }

    void CoffeMachineROSNode::bounding_boxes_callback(const darknet_ros_msgs::BoundingBoxes& msg)
    {
        //std::cout << "bounding_box" << msg << std::endl;
        cm_bounding_boxes = msg;
        //cm_raw_image = msg;
        //std::cout << cm_raw_image << std::endl;
        //pub_cm_raw_image.publish(cm_raw_image);
    }

    void CoffeMachineROSNode::openpose_hl_callback(const openpose_ros_msgs::OpenPoseHumanList& msg)
    {
        cm_openpose_hl = msg;
        std::cout << "cm_openpose_hl" << cm_openpose_hl << std::endl;
        //cm_raw_image = msg;
        //std::cout << cm_raw_image << std::endl;
        //pub_cm_raw_image.publish(cm_raw_image);
    }
    void CoffeMachineROSNode::copyImage(const sensor_msgs::ImageConstPtr& _msg)
    {
        try
        {
            img_encoding_ = _msg->encoding;//get image encodings
            cv_img_ptr_in_ = cv_bridge::toCvCopy(_msg, _msg->encoding);//get image
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("RosImgProcessorNode::image_callback(): cv_bridge exception: %s", e.what());
            return;
        }
        std::cout << "printing cv_img_ptr_in" <<  cv_img_ptr_in_ << std::endl;
    }


    void CoffeMachineROSNode::plotBoundingBoxesInImage(int x_min, int y_min, int x_max, int y_max, int id)
    {
         cv::Rect_<int> box;
         std::vector<cv::Scalar> color_classes(7);
         color_classes[0] = cv::Scalar(255,0,0); //milk
         color_classes[1] = cv::Scalar(255,0,0); //coffe
         color_classes[2] = cv::Scalar(255,0,0); //coffemaker
         color_classes[3] = cv::Scalar(255,0,0); //cup
         color_classes[4] = cv::Scalar(255,0,0); //sugar
         color_classes[5] = cv::Scalar(255,0,0); //marro_tank
         color_classes[6] = cv::Scalar(255,0,0); //water_tank

         std::vector<std::string> str_classes(7);
         str_classes[0] = "milk";
         str_classes[1] = "coffe";
         str_classes[2] = "coffemaker";
         str_classes[3] = "cup"; //red
         str_classes[4] = "sugar";
         str_classes[5] = "marro_tank";
         str_classes[6] = "water_tank";
        //check if new image is there
        if ( cv_img_ptr_in_ != nullptr )
        {
            // copy the input image to the out one
            cv_img_out_.image = cv_img_ptr_in_->image;
        }

        box.x = x_min;
        box.y = y_min;
        box.width = cvRound(x_max - x_min);
        box.height = cvRound(y_max - y_min);
        cv::rectangle(cv_img_out_.image, box, color_classes[id], 3);
         cv_img_ptr_in_ = nullptr;
        cv::putText(cv_img_out_.image,str_classes[id], cv::Point(x_min,y_min),cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
            1.0, // Scale. 2.0 = 2x bigger
            cv::Scalar(255,255,255), // BGR Color
            1); // Anti-alias (Optional)


    }

    void CoffeMachineROSNode::publishBBImage()
    {
        if( !cv_img_out_.image.data ) return;
            cv_img_out_.header.seq ++;
            cv_img_out_.header.stamp = ros::Time::now();
            cv_img_out_.header.frame_id = "camera";
            cv_img_out_.encoding = img_encoding_;
            image_pub_bb_image_out.publish(cv_img_out_.toImageMsg());
               
    }

    darknet_ros_msgs::BoundingBox CoffeMachineROSNode::find_class(const darknet_ros_msgs::BoundingBoxes& msg, std::string dk_class)
    {
        try{
            darknet_ros_msgs::BoundingBox dk_bouding_box;
            int size = msg.bounding_boxes.size();//sizeof(msg.bounding_boxes)/sizeof(msg.bounding_boxes[0]);
            float prob = 0;
            //std::cout << "size" << size << std::endl;
            for (int i = 0; i < size; i++)
            {
                //std::cout << "msg.bounding_boxes[i].Class " << msg.bounding_boxes[i].Class  << std::endl;
                //std::cout << "dk_class " << dk_class  << std::endl;
                if (msg.bounding_boxes[i].Class.compare(dk_class)==0)
                {
                    //std::cout << (msg.bounding_boxes[i].Class == dk_class) << std::endl;
                    //std::cout << "bb_prob" << (msg.bounding_boxes[i].probability) << std::endl;
                    //std::cout << "prob" << prob << std::endl;
                    if (msg.bounding_boxes[i].probability > prob)
                    {
                        prob = msg.bounding_boxes[i].probability;
                        dk_bouding_box = msg.bounding_boxes[i];
                        //std::cout << "prob" << prob << std::endl;
                        //std::cout << "dk_bouding_box2" << dk_bouding_box << std::endl;                        
                    }

                }
            }
            //std::cout << "dk_bounding_box!!!" << dk_bouding_box << std::endl;
            return dk_bouding_box;


        }
        catch(...){
             std::cout << "cannot find a bounding box" << std::endl;
        }

    }



    openpose_ros_msgs::PointWithProb CoffeMachineROSNode::find_joint(const openpose_ros_msgs::OpenPoseHumanList& msg, int num_joint)
    {
        //try
        //{
            //std::cout << "holis" << std::endl;
            openpose_ros_msgs::PointWithProb joint_point_with_prob;
            int size = msg.human_list.size();//sizeof(msg.bounding_boxes)/sizeof(msg.bounding_boxes[0]);
            //std::cout << "size" << size << std::endl;
            //std::cout << "OpenPoseHumanList" << msg << std::endl;
            if (size>0) {
                joint_point_with_prob = msg.human_list[0].body_key_points_with_prob[num_joint];
            }

            std::cout << "joint_point_with_prob" << joint_point_with_prob << std::endl;
            return joint_point_with_prob;
        //}
        //catch(...)
        //{
          //  std::cout << "cannot find the specific joint" << std::endl;
        //}
    }
    //void CoffeMachineROSNode::pub_cm_raw_image_fun(){
    //  pub_cm_raw_image.publish(cm_raw_image);
        
    //}

    BT::NodeStatus CoffeMachineROSNode::IsCleanCupReady(){

        std::cout << "IsCleanCupReady" << std::endl;
        auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/usb_cam/image_raw"); //returns a pointer to the message
        CoffeMachineROSNode::copyImage(image);
        pub_cm_raw_image.publish(image);
        darknet_ros_msgs::BoundingBox d_bb_class = CoffeMachineROSNode::find_class(cm_bounding_boxes,"cup");
        CoffeMachineROSNode::plotBoundingBoxesInImage(d_bb_class.xmin, d_bb_class.ymin, d_bb_class.xmax, d_bb_class.ymax, d_bb_class.id);
        CoffeMachineROSNode::publishBBImage();
        if (d_bb_class.probability>0.7){
            std::cout << "A cup is placed in the coffe machine" << std::endl;   
            std::cout << "bounding_box" << d_bb_class << std::endl;
            return BT::NodeStatus::SUCCESS;

        }
        else{
            return BT::NodeStatus::RUNNING;
        }






        /*auto yolo_msg = ros::topic::waitForMessage<darknet_ros_msgs::BoundingBoxes>("");
        darknet_ros_msgs::BoundingBox d_bb_class = CoffeMachineROSNode::find_class(yolo_msg,"cup");

  
        raw_image_sub = nh.subscribe("/usb_cam/image_raw",1,&CoffeMachineROSNode::raw_image_callback,this);
        openpose_ros_msgs::PointWithProb op_wrist_join = CoffeMachineROSNode::find_joint(cm_openpose_hl, 4);
        if (op_wrist_join.prob>0.7){
            std::cout << "right joint has been detected in x: "<< op_wrist_join.x << "y: " << op_wrist_join.y << std::endl; 
            return BT::NodeStatus::SUCCESS; 
        }
        else{
            return BT::NodeStatus::RUNNING;
        }

        //real one
        /*darknet_ros_msgs::BoundingBox d_bb_class = CoffeMachineROSNode::find_class(cm_bounding_boxes,"cup");
        if (d_bb_class.probability>0.7){
            std::cout << "A cup is placed in the coffe machine" << std::endl;   
            std::cout << "bounding_box" << d_bb_class << std::endl;
            return BT::NodeStatus::SUCCESS;

        }*/
        //else{
        //    return BT::NodeStatus::RUNNING;
        //}

    }

    BT::NodeStatus CoffeMachineROSNode::IsWaterTankRemoved(){
        std::cout << "IsWaterTankRemoved" << std::endl;
        //std::cout << cm_raw_image << std::endl;
        //std::cout << cm_bounding_boxes << std::endl;
        raw_image_sub = nh.subscribe("/usb_cam/image_raw",1,&CoffeMachineROSNode::raw_image_callback,this);
        darknet_ros_msgs::BoundingBox d_bb_class = CoffeMachineROSNode::find_class(cm_bounding_boxes,"water_tank");
        if (d_bb_class.probability>0.7){
            std::cout << "a water_tank is placed in the coffe machine" << std::endl;   
            std::cout << "bounding_box" << d_bb_class << std::endl;
            return BT::NodeStatus::SUCCESS;

        }
        else{
            return BT::NodeStatus::RUNNING;
        }

        //std::cout << "cm_openpose_hl" << op_wrist_join << std::endl;
 
        //if (d_bb_class.probability>0.7){}

        //}
    
    return BT::NodeStatus::RUNNING;

    }

    
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

        //BT::NodeStatus IsWaterTankRemoved()
    //{
        //status = global.AskForStatus("IsWaterTankRemoved");
        //return status;
        //return BT::NodeStatus::SUCCESS;

    //}

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