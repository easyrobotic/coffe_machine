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
        //raw_image_sub = nh.subscribe("/camera/rgb/image_rect_color",1,&CoffeMachineROSNode::raw_image_callback,this);
        bounding_boxes_sub = nh.subscribe("/darknet_ros/bounding_boxes",100,&CoffeMachineROSNode::bounding_boxes_callback,this);
        openpose_sub = nh.subscribe("/openpose_ros/human_list",100,&CoffeMachineROSNode::openpose_hl_callback,this);
    }

    void CoffeMachineROSNode::initializePublishers()
    {
        ROS_INFO("Initializing Publishers");
        pub_cm_raw_image = nh.advertise<sensor_msgs::Image>("/coffe_machine/image_raw", 100);  
        image_transport::ImageTransport img_tp_(nh); 
        image_pub_bb_image_out = img_tp_.advertise("/coffe_machine/image_with_output_data", 100);
        bh_tree = nh.advertise<coffe_machine::State>("/coffe_machine/Feedback", 100);  
    }
  

    void CoffeMachineROSNode::raw_image_callback(const sensor_msgs::Image& msg)
    {
        cm_raw_image = msg;
        pub_cm_raw_image.publish(cm_raw_image);
        //auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_rect_color"); //returns a pointer to the message
        //CoffeMachineROSNode::copyImage(cm_raw_image);
        //pub_cm_raw_image.publish(cm_raw_image);
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
    void CoffeMachineROSNode::copyImage(const sensor_msgs::ImageConstPtr& _msg)
    {
        try
        {
            img_encoding_ = _msg->encoding;//get image encodings
            cv_img_ptr_in_ = cv_bridge::toCvCopy(_msg, _msg->encoding);//get image
            /*if ( cv_img_ptr_in_ != nullptr )
            {
                // copy the input image to the out one
                std::cout << "cv_img_out" << cv_img_out_ << std::endl;
                cv_img_out_.image = cv_img_ptr_in_->image;
            }*/
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("RosImgProcessorNode::image_callback(): cv_bridge exception: %s", e.what());
            return;
        }
        //std::cout << "printing cv_img_ptr_in" <<  cv_img_ptr_in_ << std::endl;
    }


    void CoffeMachineROSNode::plotBoundingBoxesInImage(int x_min, int y_min, int x_max, int y_max, int id)
    {
         cv::Rect_<int> box;
         std::vector<cv::Scalar> color_classes(7);
         color_classes[0] = cv::Scalar(255,0,0); //milk
         color_classes[1] = cv::Scalar(255,0,0); //coffe
         color_classes[2] = cv::Scalar(0,0,0); //coffemaker
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
        if(id!=-1){
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


    }
    void CoffeMachineROSNode::plotJointInImage(int x, int y, int joint)
    {
         //std::cout << "holis0" << std::endl;
         std::vector<std::string> name_joint(8);
         name_joint[0] = "nouse"; //milk
         name_joint[1] = "neck"; //coffe
         name_joint[2] = "right_shoulder"; //coffemaker
         name_joint[3] = "right_elbow"; //cup
         name_joint[4] = "right_wrist"; //sugar
         name_joint[5] = "left_shoulder"; //marro_tank
         name_joint[6] = "left_elbow"; //water_tank
         name_joint[7] = "left_wrist"; //water_tank

         //std::cout << "holis1" << std::endl;
        //check if new image is there
        if ( cv_img_ptr_in_ != nullptr )
        {
            // copy the input image to the out one
            cv_img_out_.image = cv_img_ptr_in_->image;
        }
        //std::cout << "holis2" << std::endl;
        cv::circle(cv_img_out_.image, cv::Point(x, y), 10, CV_RGB(0, 255, 0), -1, CV_AA);
        //std::cout << "holis3" << std::endl;
        cv::putText(cv_img_out_.image,name_joint[joint], cv::Point(x,y),cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
            1.0, // Scale. 2.0 = 2x bigger
            cv::Scalar(255,255,255), // BGR Color
            1); // Anti-alias (Optional)
        //std::cout << "holis4" << std::endl;    


    }

    void CoffeMachineROSNode::publishCmRawImage(){
        auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_rect_color"); //returns a pointer to the message
        CoffeMachineROSNode::copyImage(image);
        pub_cm_raw_image.publish(image);
    }

    void CoffeMachineROSNode::publishBBImage()
    {
        if( !cv_img_out_.image.data ) return;
            cv_img_out_.header.seq ++;
            cv_img_out_.header.stamp = ros::Time::now();
            cv_img_out_.header.frame_id = "camera";
            cv_img_out_.encoding = img_encoding_;
            cv::imshow("image_output", cv_bridge::toCvShare(cv_img_out_.toImageMsg(), "bgr8")->image);
            image_pub_bb_image_out.publish(cv_img_out_.toImageMsg());
            cv::waitKey(1000);
               
    }

     void CoffeMachineROSNode::publishBTState(std::string NodeType, std::string NodeStatus)
    {
        coffe_machine::State cm_state_msg;
        cm_state_msg.NodeType = NodeType;
        cm_state_msg.NodeStatus = NodeStatus;    
        bh_tree.publish(cm_state_msg);           
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


    

   /* START BT CONDITIONS, ACTIONS*/

   /***************************************************************OpenCoffeMachine Subtree****************************************************************************/


   BT::NodeStatus CoffeMachineROSNode::IsMachineOpen()
    {
        status = global.AskForStatus("IsMachineOpen");
        //CoffeMachineROSNode::publishBBImage();
        if (status == BT::NodeStatus::SUCCESS)
        {
            publishBTState("IsMachineOpen", "SUCCESS");            
        }
        else if (status == BT::NodeStatus::RUNNING)
        {
            publishBTState("IsMachineOpen", "RUNNING");            
        }

        else if (status == BT::NodeStatus::FAILURE)
        {
            publishBTState("IsMachineOpen", "FAILURE");            
        }

        return status;
        //return BT::NodeStatus::SUCCESS;
    }





    BT::NodeStatus CoffeMachineROSNode::IsCleanCupReady(){

        std::cout << "IsCleanCupReady" << std::endl;
        CoffeMachineROSNode::publishCmRawImage();
        /*auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_rect_color"); //returns a pointer to the message
        CoffeMachineROSNode::copyImage(image);
        pub_cm_raw_image.publish(image);*/
        darknet_ros_msgs::BoundingBox d_bb_cup = CoffeMachineROSNode::find_class(cm_bounding_boxes,"cup");
        darknet_ros_msgs::BoundingBox d_bb_coffeemaker = CoffeMachineROSNode::find_class(cm_bounding_boxes,"coffeemaker");

        if ((d_bb_cup.probability>0.6) and (d_bb_coffeemaker.probability>0.1)){
            if (d_bb_cup.probability>0.6)
            {
                std::cout << "A cup is placed in the coffe machine" << std::endl;   
                std::cout << "bounding_box_cup" << d_bb_cup << std::endl;
                CoffeMachineROSNode::plotBoundingBoxesInImage(d_bb_cup.xmin, d_bb_cup.ymin, d_bb_cup.xmax, d_bb_cup.ymax, d_bb_cup.id);
            }
            if (d_bb_coffeemaker.probability>0.1) 
            {
                std::cout << "The coffe machine has been detected" << std::endl;   
                std::cout << "bounding_box_coffeemaker" << d_bb_coffeemaker << std::endl;        
                CoffeMachineROSNode::plotBoundingBoxesInImage(d_bb_coffeemaker.xmin, d_bb_coffeemaker.ymin, d_bb_coffeemaker.xmax, d_bb_coffeemaker.ymax, d_bb_coffeemaker.id);
            }

            CoffeMachineROSNode::publishBBImage();
            publishBTState("IsCleanCupReady", "SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }
        else{
            //CoffeMachineROSNode::plotBoundingBoxesInImage( 0, 0, 0 , 0, -1);
            //CoffeMachineROSNode::publishBBImage();
            publishBTState("IsCleanCupReady", "RUNNING");
            return BT::NodeStatus::RUNNING;
        }

    }

    BT::NodeStatus CoffeMachineROSNode::SwitchOnCoffeMachine()
    {
        //_opened = true;
        status = global.AskForStatus("SwitchOnCoffeMachine");
        if (status == BT::NodeStatus::SUCCESS)
        {
            publishBTState("SwitchOnCoffeMachine", "SUCCESS");            
        }
        else if (status == BT::NodeStatus::RUNNING)
        {
            publishBTState("SwitchOnCoffeMachine", "RUNNING");            
        }

        else if (status == BT::NodeStatus::FAILURE)
        {
            publishBTState("SwitchOnCoffeMachine", "FAILURE");            
        }
        return status;
        //return BT::NodeStatus::SUCCESS;
    }

    
    
    
    
    /*******************************************AutoClean Subtree******************************************************/

     BT::NodeStatus CoffeMachineROSNode::IsCleanProcessFinished()
    {
        status = global.AskForStatus("IsCleanProcessFinished");
        if (status == BT::NodeStatus::SUCCESS)
        {
            publishBTState("IsCleanProcessFinished", "SUCCESS");            
        }
        else if (status == BT::NodeStatus::RUNNING)
        {
            publishBTState("IsCleanProcessFinished", "RUNNING");            
        }

        else if (status == BT::NodeStatus::FAILURE)
        {
            publishBTState("IsCleanProcessFinished", "FAILURE");            
        }

        return status;
        //return BT::NodeStatus::SUCCESS;

    }

    BT::NodeStatus CoffeMachineROSNode::IsThereEnoughWater()
    {
        status = global.AskForStatus("IsThereEnoughWater");
        if (status == BT::NodeStatus::SUCCESS)
        {
            publishBTState("IsThereEnoughWater", "SUCCESS");            
        }
        else if (status == BT::NodeStatus::RUNNING)
        {
            publishBTState("IsThereEnoughWater", "RUNNING");            
        }

        else if (status == BT::NodeStatus::FAILURE)
        {
            publishBTState("IsThereEnoughWater", "FAILURE");            
        }        
        return status;
        //return BT::NodeStatus::SUCCESS;

    }


    BT::NodeStatus CoffeMachineROSNode::IsWaterTankRemoved(){
        std::cout << "IsWaterTankRemoved" << std::endl;
        CoffeMachineROSNode::publishCmRawImage();
        darknet_ros_msgs::BoundingBox d_bb_water_tank = CoffeMachineROSNode::find_class(cm_bounding_boxes,"water_tank");
        openpose_ros_msgs::PointWithProb op_wrist_join = CoffeMachineROSNode::find_joint(cm_openpose_hl, 4);
        if (op_wrist_join.prob>0.6)
        {
            std::cout << "op_wrist_joint" << op_wrist_join << std::endl;
            CoffeMachineROSNode::plotJointInImage(op_wrist_join.x, op_wrist_join.y, 4);
            CoffeMachineROSNode::publishBBImage();
            publishBTState("IsWaterTankRemoved", "SUCCESS"); //!!!!!Needs to be removed as soon as the tank is detected!!!!!
            return BT::NodeStatus::SUCCESS;
        }

        if (d_bb_water_tank.probability>0.2){
            std::cout << "a water_tank is placed in the coffe machine" << std::endl;   
            std::cout << "bounding_box" << d_bb_water_tank << std::endl;
            CoffeMachineROSNode::plotBoundingBoxesInImage(d_bb_water_tank.xmin, d_bb_water_tank.ymin, d_bb_water_tank.xmax, d_bb_water_tank.ymax, d_bb_water_tank.id);

            CoffeMachineROSNode::publishBBImage();
            publishBTState("IsWaterTankRemoved", "SUCCESS");
            return BT::NodeStatus::SUCCESS;

        }
        else{
            publishBTState("IsWaterTankRemoved", "RUNNING");
            //return BT::NodeStatus::RUNNING;
        }

    
        return BT::NodeStatus::RUNNING;

    }


    BT::NodeStatus CoffeMachineROSNode::IsWaterTankFull()
    {
        status = global.AskForStatus("IsWaterTankFull");
        if (status == BT::NodeStatus::SUCCESS)
        {
            publishBTState("IsWaterTankFull", "SUCCESS");            
        }
        else if (status == BT::NodeStatus::RUNNING)
        {
            publishBTState("IsWaterTankFull", "RUNNING");            
        }

        else if (status == BT::NodeStatus::FAILURE)
        {
            publishBTState("IsWaterTankFull", "FAILURE");            
        }        
        
        return status;

    }

    
    BT::NodeStatus CoffeMachineROSNode::FillWaterTank()
    {
        //_filled = true;
        status = global.AskForStatus("FillWaterTank");
        if (status == BT::NodeStatus::SUCCESS)
        {
            publishBTState("FillWaterTank", "SUCCESS");            
        }
        else if (status == BT::NodeStatus::RUNNING)
        {
            publishBTState("FillWaterTank", "RUNNING");            
        }

        else if (status == BT::NodeStatus::FAILURE)
        {
            publishBTState("FillWaterTank", "FAILURE");            
        }        
        return status;
    }


       BT::NodeStatus CoffeMachineROSNode::IsWaterTankPlacedInCoffeMachine()
    {
        status = global.AskForStatus("IsWaterTankPlacedInCoffeMachine");
        if (status == BT::NodeStatus::SUCCESS)
        {
            publishBTState("IsWaterTankPlacedInCoffeMachine", "SUCCESS");            
        }
        else if (status == BT::NodeStatus::RUNNING)
        {
            publishBTState("IsWaterTankPlacedInCoffeMachine", "RUNNING");            
        }

        else if (status == BT::NodeStatus::FAILURE)
        {
            publishBTState("IsWaterTankPlacedInCoffeMachine", "FAILURE");            
        }        
        
        return status;

    }


       BT::NodeStatus CoffeMachineROSNode::IsMarroTankFull()
    {
        status = global.AskForStatus("IsMarroTankFull");
        if (status == BT::NodeStatus::SUCCESS)
        {
            publishBTState("IsMarroTankFull", "SUCCESS");            
        }
        else if (status == BT::NodeStatus::RUNNING)
        {
            publishBTState("IsMarroTankFull", "RUNNING");            
        }

        else if (status == BT::NodeStatus::FAILURE)
        {
            publishBTState("IsMarroTankFull", "FAILURE");            
        }        
        
        return status;

    }
       BT::NodeStatus CoffeMachineROSNode::IsMarroTankRemoved()
    {
        std::cout << "IsMarroTankRemoved" << std::endl;
        CoffeMachineROSNode::publishCmRawImage();
        darknet_ros_msgs::BoundingBox d_bb_marro_tank = CoffeMachineROSNode::find_class(cm_bounding_boxes,"marro_tank");
        std::cout << "test1" << std::endl;
        openpose_ros_msgs::PointWithProb op_wrist_join = CoffeMachineROSNode::find_joint(cm_openpose_hl, 4);
        std::cout << "test2" << std::endl;
        std::cout << "op_wrist_join" << op_wrist_join << std::endl;
        if (op_wrist_join.prob>0.6)
        {
            std::cout << "op_wrist_joint" << op_wrist_join << std::endl;
            CoffeMachineROSNode::plotJointInImage(op_wrist_join.x, op_wrist_join.y, 4);
            std::cout << "test3" << std::endl;
            CoffeMachineROSNode::publishBBImage();
            publishBTState("IsMarroTankRemoved", "SUCCESS"); //!!!!!Needs to be removed as soon as the tank is detected!!!!!
            return BT::NodeStatus::SUCCESS;
        }
            std::cout << "test6" << std::endl;
        if (d_bb_marro_tank.probability>0.2){
            std::cout << "test4" << std::endl;
            std::cout << "a marro_tank is placed in the coffe machine" << std::endl;   
            std::cout << "bounding_box" << d_bb_marro_tank << std::endl;
            CoffeMachineROSNode::plotBoundingBoxesInImage(d_bb_marro_tank.xmin, d_bb_marro_tank.ymin, d_bb_marro_tank.xmax, d_bb_marro_tank.ymax, d_bb_marro_tank.id);
            std::cout << "test5" << std::endl;
            CoffeMachineROSNode::publishBBImage();
            publishBTState("IsMarroTankRemoved", "SUCCESS");
            return BT::NodeStatus::SUCCESS;
            std::cout << "test7" << std::endl;
        }
        else{
            publishBTState("IsMarroTankRemoved", "RUNNING");
            return BT::NodeStatus::RUNNING;
        }
            std::cout << "test8" << std::endl;
    
        return BT::NodeStatus::RUNNING;


    }

        BT::NodeStatus CoffeMachineROSNode::IsMarroTankEmpty()
    {
        status = global.AskForStatus("IsMarroTankEmpty");
        if (status == BT::NodeStatus::SUCCESS)
        {
            publishBTState("IsMarroTankEmpty", "SUCCESS");            
        }
        else if (status == BT::NodeStatus::RUNNING)
        {
            publishBTState("IsMarroTankEmpty", "RUNNING");            
        }

        else if (status == BT::NodeStatus::FAILURE)
        {
            publishBTState("IsMarroTankEmpty", "FAILURE");            
        }        
        return status;

    }

       BT::NodeStatus CoffeMachineROSNode::EmptyMarroTank()
    {
       // _empty = true;
        status = global.AskForStatus("EmptyMarroTank");
        if (status == BT::NodeStatus::SUCCESS)
        {
            publishBTState("EmptyMarroTank", "SUCCESS");            
        }
        else if (status == BT::NodeStatus::RUNNING)
        {
            publishBTState("EmptyMarroTank", "RUNNING");            
        }

        else if (status == BT::NodeStatus::FAILURE)
        {
            publishBTState("EmptyMarroTank", "FAILURE");            
        }        
        return status;
    }

        BT::NodeStatus CoffeMachineROSNode::IsMarroTankPlacedInCoffeMachine()
    {

        status = global.AskForStatus("IsMarroTankPlacedInCoffeMachine");
        if (status == BT::NodeStatus::SUCCESS)
        {
            publishBTState("IsMarroTankPlacedInCoffeMachine", "SUCCESS");            
        }
        else if (status == BT::NodeStatus::RUNNING)
        {
            publishBTState("IsMarroTankPlacedInCoffeMachine", "RUNNING");            
        }

        else if (status == BT::NodeStatus::FAILURE)
        {
            publishBTState("IsMarroTankPlacedInCoffeMachine", "FAILURE");            
        }        
        return status;

    }

        /*******************************************PutCoffeCup Subtree******************************************************/


        BT::NodeStatus CoffeMachineROSNode::IsCoffeCupReady()
    {
        std::cout << "IsCoffeCupReady" << std::endl;
        CoffeMachineROSNode::publishCmRawImage();
        darknet_ros_msgs::BoundingBox d_bb_cup = CoffeMachineROSNode::find_class(cm_bounding_boxes,"cup");
        darknet_ros_msgs::BoundingBox d_bb_coffeemaker = CoffeMachineROSNode::find_class(cm_bounding_boxes,"coffeemaker");

        if ((d_bb_cup.probability>0.8) and (d_bb_coffeemaker.probability>0.1)){
            if (d_bb_cup.probability>0.8)
            {
                std::cout << "A cup is placed in the coffe machine" << std::endl;   
                std::cout << "bounding_box_cup" << d_bb_cup << std::endl;
                CoffeMachineROSNode::plotBoundingBoxesInImage(d_bb_cup.xmin, d_bb_cup.ymin, d_bb_cup.xmax, d_bb_cup.ymax, d_bb_cup.id);
            }
            if (d_bb_coffeemaker.probability>0.1) 
            {
                std::cout << "The coffe machine has been detected" << std::endl;   
                std::cout << "bounding_box_coffeemaker" << d_bb_coffeemaker << std::endl;        
                CoffeMachineROSNode::plotBoundingBoxesInImage(d_bb_coffeemaker.xmin, d_bb_coffeemaker.ymin, d_bb_coffeemaker.xmax, d_bb_coffeemaker.ymax, d_bb_coffeemaker.id);
            }

            CoffeMachineROSNode::publishBBImage();
            publishBTState("IsCleanCupReady", "SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }
        else{
            //CoffeMachineROSNode::plotBoundingBoxesInImage( 0, 0, 0 , 0, -1);
            //CoffeMachineROSNode::publishBBImage();
            publishBTState("IsCleanCupReady", "RUNNING");
            return BT::NodeStatus::RUNNING;
        }



    }

        BT::NodeStatus CoffeMachineROSNode::PlaceCoffeCup()
        {
        //_coffecuplaced = true;
        status = global.AskForStatus("PlaceCoffeCup");
        if (status == BT::NodeStatus::SUCCESS)
        {
            publishBTState("PlaceCoffeCup", "SUCCESS");            
        }
        else if (status == BT::NodeStatus::RUNNING)
        {
            publishBTState("PlaceCoffeCup", "RUNNING");            
        }

        else if (status == BT::NodeStatus::FAILURE)
        {
            publishBTState("PlaceCoffeCup", "FAILURE");            
        }        
        return status;

        }

        /*******************************************CofeeType Subtree******************************************************/


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