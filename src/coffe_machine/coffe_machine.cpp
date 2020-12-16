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
        //bounding_boxes_sub = nh.subscribe("/darknet_ros/bounding_boxes",100,&CoffeMachineROSNode::bounding_boxes_callback,this);
        //openpose_sub = nh.subscribe("/openpose_ros/human_list",100,&CoffeMachineROSNode::openpose_hl_callback,this);
    }

    void CoffeMachineROSNode::initializePublishers()
    {
        ROS_INFO("Initializing Publishers");
        pub_yolo_image_raw = nh.advertise<sensor_msgs::Image>("/coffe_machine/yolo_image_raw", 100);  
        pub_openpose_image_raw = nh.advertise<sensor_msgs::Image>("/coffe_machine/openpose_image_raw", 100);  
        image_transport::ImageTransport img_tp_(nh); 
        image_pub_bb_image_out = img_tp_.advertise("/coffe_machine/image_with_output_data", 100);
        bh_tree = nh.advertise<coffe_machine::State>("/coffe_machine/Feedback", 100);  
    }
  

    /*void CoffeMachineROSNode::raw_image_callback(const sensor_msgs::Image& msg)
    {
        cm_raw_image = msg;
        pub_cm_raw_image.publish(cm_raw_image);
        //auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_rect_color"); //returns a pointer to the message
        //CoffeMachineROSNode::copyImage(cm_raw_image);
        //pub_cm_raw_image.publish(cm_raw_image);
    }*/

    /*void CoffeMachineROSNode::bounding_boxes_callback(const darknet_ros_msgs::BoundingBoxes& msg)
    {
        //std::cout << "bounding_box" << msg << std::endl;
        cm_bounding_boxes = msg;
        //cm_raw_image = msg;
        //std::cout << cm_raw_image << std::endl;
        //pub_cm_raw_image.publish(cm_raw_image);
    }*/

    /*void CoffeMachineROSNode::openpose_hl_callback(const openpose_ros_msgs::OpenPoseHumanList& msg)
    {
        cm_openpose_hl = msg;
        //std::cout << "cm_openpose_hl" << cm_openpose_hl << std::endl;
        //cm_raw_image = msg;
        //std::cout << cm_raw_image << std::endl;
        //pub_cm_raw_image.publish(cm_raw_image);
    }*/

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


    void CoffeMachineROSNode::plotBoundingBoxesInImage(int x_min, int y_min, int x_max, int y_max, int id,cv_bridge::CvImagePtr image)
    {
         cv::Rect_<int> box;
         std::vector<cv::Scalar> color_classes(7);
         color_classes[0] = cv::Scalar(255,255,255); //milk
         color_classes[1] = cv::Scalar(128,64,0); //coffe
         color_classes[2] = cv::Scalar(0,0,0); //coffemaker
         color_classes[3] = cv::Scalar(255,0,0); //cup
         color_classes[4] = cv::Scalar(246,210,88); //sugar
         color_classes[5] = cv::Scalar(90,53,47); //marro_tank
         color_classes[6] = cv::Scalar(0,128,255); //water_tank

         std::vector<std::string> str_classes(7);
         str_classes[0] = "milk";
         str_classes[1] = "coffe";
         str_classes[2] = "coffemaker";
         str_classes[3] = "cup"; //red
         str_classes[4] = "sugar";
         str_classes[5] = "marro_tank";
         str_classes[6] = "water_tank";

        ////check if new image is there
        //if ( cv_img_ptr_in_ != nullptr )
          if ( image != nullptr )
        {
            // copy the input image to the out one
            cv_img_out_.image = image->image;
        }
        if(id!=-1){
            box.x = x_min;
            box.y = y_min;
            box.width = cvRound(x_max - x_min);
            box.height = cvRound(y_max - y_min);
            cv::rectangle(cv_img_out_.image, box, color_classes[id], 3);
            cv_img_ptr_in_ = nullptr;
            cv::putText(cv_img_out_.image,str_classes[id], cv::Point(x_min,y_min),cv::FONT_HERSHEY_DUPLEX, // Font
                1.0, // Scale. 2.0 = 2x bigger
                cv::Scalar(255,255,255), // BGR Color
                1); // Anti-alias (Optional)

            /*cv::circle(cv_img_out_.image, cv::Point(x_min, y_min), 5, CV_RGB(255, 0, 0), -1, CV_AA);
            cv::circle(cv_img_out_.image, cv::Point(x_min, y_max), 5, CV_RGB(255, 0, 0), -1, CV_AA);
            cv::circle(cv_img_out_.image, cv::Point(x_max, y_min), 5, CV_RGB(255, 0, 0), -1, CV_AA);
            cv::circle(cv_img_out_.image, cv::Point(x_max, y_max), 5, CV_RGB(255, 0, 0), -1, CV_AA);*/
        }


    }
    void CoffeMachineROSNode::plotJointInImage(int x, int y, int joint,cv_bridge::CvImagePtr image)
    {
         //std::cout << "holis0" << std::endl;
         std::vector<std::string> name_joint(8);
         name_joint[0] = "nouse"; 
         name_joint[1] = "neck"; 
         name_joint[2] = "right_shoulder"; 
         name_joint[3] = "right_elbow"; 
         name_joint[4] = "right_wrist"; 
         name_joint[5] = "left_shoulder"; 
         name_joint[6] = "left_elbow"; 
         name_joint[7] = "left_wrist"; 

        std::vector<cv::Scalar> color_classes(8);
         color_classes[0] = cv::Scalar(254,127,156); 
         color_classes[1] = cv::Scalar(202,0,42); 
         color_classes[2] = cv::Scalar(255,105,0); 
         color_classes[3] = cv::Scalar(255,211,0); 
         color_classes[4] = cv::Scalar(255,255,0); 
         color_classes[5] = cv::Scalar(57,255,20); 
         color_classes[6] = cv::Scalar(76,187,23); 
         color_classes[7] = cv::Scalar(11,102,35); 

         //std::cout << "holis1" << std::endl;
        //check if new image is there
        //if ( cv_img_ptr_in_ != nullptr )
        if ( image != nullptr )
        {
            // copy the input image to the out one
            //cv_img_out_.image = cv_img_ptr_in_->image;
            cv_img_out_.image = image->image;
        }
        //std::cout << "holis2" << std::endl;
        cv::circle(cv_img_out_.image, cv::Point(x, y), 10, color_classes[joint], -1, CV_AA);
        //std::cout << "holis3" << std::endl;
        cv::putText(cv_img_out_.image,name_joint[joint], cv::Point(x,y),cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
            1.0, // Scale. 2.0 = 2x bigger
            cv::Scalar(255,255,255), // BGR Color
            1); // Anti-alias (Optional)
        //std::cout << "holis4" << std::endl;    


    }

    /*void CoffeMachineROSNode::publishCmRawImage(){
        auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_rect_color"); //returns a pointer to the message
        CoffeMachineROSNode::copyImage(image);
        pub_cm_raw_image.publish(image);
    }*/

    void CoffeMachineROSNode::publishBBImage(std::string image_name)
    {
        if( !cv_img_out_.image.data ) return;
            cv_img_out_.header.seq ++;
            cv_img_out_.header.stamp = ros::Time::now();
            cv_img_out_.header.frame_id = "camera";
            cv_img_out_.encoding = img_encoding_;
            std::cout << "holis9" << std::endl;
            std::string path_ = "/home/julia/tfm/src/coffe_machine/output_img/";   
            cv::imwrite(path_+image_name, cv_bridge::toCvShare(cv_img_out_.toImageMsg(), "bgr8")->image) ;
            //cv::imshow("image_output", cv_bridge::toCvShare(cv_img_out_.toImageMsg(), "bgr8")->image);
            image_pub_bb_image_out.publish(cv_img_out_.toImageMsg());
            //cv::waitKey(1000); //save the image and go ahead
               
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

        //bool image_sent = false;
        std::cout << "IsCleanCupReady" << std::endl;
        //CoffeMachineROSNode::publishCmRawImage();
        //if (not (image_sent)){
        auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_rect_color"); //returns a pointer to the message
        CoffeMachineROSNode::copyImage(image);
        pub_yolo_image_raw.publish(image);
        img_encoding_ = image->encoding;//get image encodings
        cv_bridge::CvImagePtr image_in = cv_bridge::toCvCopy(image, image->encoding);//get image
        //image_sent = true;
        
        
        auto yolo_msg = ros::topic::waitForMessage<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes");
        darknet_ros_msgs::BoundingBox d_bb_cup = CoffeMachineROSNode::find_class(*yolo_msg,"cup");
        darknet_ros_msgs::BoundingBox d_bb_coffeemaker = CoffeMachineROSNode::find_class(*yolo_msg,"coffeemaker");

        if (d_bb_coffeemaker.probability>0.1)
        {
            if (d_bb_cup.probability>0.6)
            {
                    std::cout << "A cup is placed in the coffe machine" << std::endl;   
                    CoffeMachineROSNode::plotBoundingBoxesInImage(d_bb_cup.xmin, d_bb_cup.ymin, d_bb_cup.xmax, d_bb_cup.ymax, d_bb_cup.id,image_in);     
                    CoffeMachineROSNode::plotBoundingBoxesInImage(d_bb_coffeemaker.xmin, d_bb_coffeemaker.ymin, d_bb_coffeemaker.xmax, d_bb_coffeemaker.ymax, d_bb_coffeemaker.id,image_in);
                    CoffeMachineROSNode::publishBBImage("1.Clean_cup_is_ready.jpg");
                    publishBTState("IsCleanCupReady", "SUCCESS");
                    return BT::NodeStatus::SUCCESS;
            
            }
            else{
                    publishBTState("IsCleanCupReady", "FAILURE");
                    return BT::NodeStatus::FAILURE;
            }
        }
        else{
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
        auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_rect_color"); 
        CoffeMachineROSNode::copyImage(image);
        pub_yolo_image_raw.publish(image);
        img_encoding_ = image->encoding;//get image encodings
        cv_bridge::CvImagePtr image_in = cv_bridge::toCvCopy(image, image->encoding);//get image

        auto yolo_msg = ros::topic::waitForMessage<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes");
        darknet_ros_msgs::BoundingBox d_bb_water_tank = CoffeMachineROSNode::find_class(*yolo_msg,"water_tank");
        if (d_bb_water_tank.probability>0.2){
            pub_openpose_image_raw.publish(image);
            auto openpose_msg = ros::topic::waitForMessage<openpose_ros_msgs::OpenPoseHumanList>("/openpose_ros/human_list"); 
            openpose_ros_msgs::PointWithProb op_wrist_join_right = CoffeMachineROSNode::find_joint(*openpose_msg, 4);
            openpose_ros_msgs::PointWithProb op_wrist_join_left = CoffeMachineROSNode::find_joint(*openpose_msg, 7);
            if ((op_wrist_join_right.prob>0)and(op_wrist_join_left.prob>0)){
                CoffeMachineROSNode::plotBoundingBoxesInImage(d_bb_water_tank.xmin, d_bb_water_tank.ymin, d_bb_water_tank.xmax, d_bb_water_tank.ymax, d_bb_water_tank.id,image_in);
                CoffeMachineROSNode::plotJointInImage(op_wrist_join_right.x, op_wrist_join_right.y, 4, image_in);
                CoffeMachineROSNode::plotJointInImage(op_wrist_join_left.x, op_wrist_join_left.y, 7, image_in);
                if 
                fabs(x - 0.5 * (xmin+xmax)) < (0.5 *  (xmax-xmin) + dist)

                CoffeMachineROSNode::publishBBImage("2.Human_handling_water_tank.jpg");
                publishBTState("IsWaterTankRemoved", "SUCCESS");
                return BT::NodeStatus::SUCCESS;

            }
            else{
                publishBTState("IsWaterTankRemoved", "FAILURE");
                return BT::NodeStatus::FAILURE;
            }
        }
        else{
                publishBTState("IsWaterTankRemoved", "RUNNING");
                return BT::NodeStatus::RUNNING;            
        }

    }

    //certs nodes picar enter per procedir.


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
        auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_rect_color"); 
        CoffeMachineROSNode::copyImage(image);
        pub_yolo_image_raw.publish(image);
        img_encoding_ = image->encoding;//get image encodings
        cv_img_ptr_in_ = cv_bridge::toCvCopy(image, image->encoding);//get image

        auto yolo_msg = ros::topic::waitForMessage<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes");
        darknet_ros_msgs::BoundingBox d_bb_marro_tank = CoffeMachineROSNode::find_class(*yolo_msg,"marro_tank");
        if (d_bb_marro_tank.probability>0.2){
            pub_openpose_image_raw.publish(image);
            auto openpose_msg = ros::topic::waitForMessage<openpose_ros_msgs::OpenPoseHumanList>("/openpose_ros/human_list"); 
            openpose_ros_msgs::PointWithProb op_wrist_join = CoffeMachineROSNode::find_joint(*openpose_msg, 4);
            if (op_wrist_join.prob>0){
                CoffeMachineROSNode::plotBoundingBoxesInImage(d_bb_marro_tank.xmin, d_bb_marro_tank.ymin, d_bb_marro_tank.xmax, d_bb_marro_tank.ymax, d_bb_marro_tank.id,cv_img_ptr_in_);
                CoffeMachineROSNode::plotJointInImage(op_wrist_join.x, op_wrist_join.y, 4,cv_img_ptr_in_);
                CoffeMachineROSNode::publishBBImage("3.Human_handling_marro_tank");
                publishBTState("IsMarroTankRemoved", "SUCCESS");
                return BT::NodeStatus::SUCCESS;

            }
            else{
                publishBTState("IsMarroTankRemoved", "SUCCESS");
                return BT::NodeStatus::FAILURE;
            }
        }
        else{
                publishBTState("IsMarroTankRemoved", "RUNNING");
                return BT::NodeStatus::RUNNING;            
        }



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
        //CoffeMachineROSNode::publishCmRawImage();
        //if (not (image_sent)){
        auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_rect_color"); //returns a pointer to the message
        CoffeMachineROSNode::copyImage(image);
        pub_yolo_image_raw.publish(image);
        img_encoding_ = image->encoding;//get image encodings
        cv_img_ptr_in_ = cv_bridge::toCvCopy(image, image->encoding);//get image
        //image_sent = true;
        
        
        auto yolo_msg = ros::topic::waitForMessage<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes");
        darknet_ros_msgs::BoundingBox d_bb_cup = CoffeMachineROSNode::find_class(*yolo_msg,"cup");
        darknet_ros_msgs::BoundingBox d_bb_coffeemaker = CoffeMachineROSNode::find_class(*yolo_msg,"coffeemaker");

        if (d_bb_coffeemaker.probability>0.1)
        {
            if (d_bb_cup.probability>0.6)
            {
                    std::cout << "A cup is placed in the coffe machine" << std::endl;   
                    CoffeMachineROSNode::plotBoundingBoxesInImage(d_bb_cup.xmin, d_bb_cup.ymin, d_bb_cup.xmax, d_bb_cup.ymax, d_bb_cup.id,cv_img_ptr_in_);     
                    CoffeMachineROSNode::plotBoundingBoxesInImage(d_bb_coffeemaker.xmin, d_bb_coffeemaker.ymin, d_bb_coffeemaker.xmax, d_bb_coffeemaker.ymax, d_bb_coffeemaker.id,cv_img_ptr_in_);
                    CoffeMachineROSNode::publishBBImage("4.Coffe_Cup_is_Ready");
                    publishBTState("IsCoffeCupReady", "SUCCESS");
                    return BT::NodeStatus::SUCCESS;
            
            }
            else{
                    publishBTState("IsCoffeCupReady", "FAILURE");
                    return BT::NodeStatus::FAILURE;
            }
        }
        else{
            publishBTState("IsCoffeCupReady", "RUNNING");
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

        BT::NodeStatus CoffeMachineROSNode::IsDesiredCoffeSelected()
    {
        status = global.AskForStatus("IsDesiredCoffeSelected");
        if (status == BT::NodeStatus::SUCCESS)
        {
            publishBTState("IsDesiredCoffeSelected", "SUCCESS");            
        }
        else if (status == BT::NodeStatus::RUNNING)
        {
            publishBTState("IsDesiredCoffeSelected", "RUNNING");            
        }

        else if (status == BT::NodeStatus::FAILURE)
        {
            publishBTState("IsDesiredCoffeSelected", "FAILURE");            
        }        
        return status;

    }


        BT::NodeStatus CoffeMachineROSNode::PressDesiredCoffe()
    {
        //_desiredcoffepressed = true;
        status = global.AskForStatus("PressDesiredCoffe");
        if (status == BT::NodeStatus::SUCCESS)
        {
            publishBTState("PressDesiredCoffe", "SUCCESS");            
        }
        else if (status == BT::NodeStatus::RUNNING)
        {
            publishBTState("PressDesiredCoffe", "RUNNING");            
        }

        else if (status == BT::NodeStatus::FAILURE)
        {
            publishBTState("PressDesiredCoffe", "FAILURE");            
        }        
        return status;
    }

        BT::NodeStatus CoffeMachineROSNode::IsCoffeFinished()
    {
        status = global.AskForStatus("IsCoffeFinished");
        if (status == BT::NodeStatus::SUCCESS)
        {
            publishBTState("IsCoffeFinished", "SUCCESS");            
        }
        else if (status == BT::NodeStatus::RUNNING)
        {
            publishBTState("IsCoffeFinished", "RUNNING");            
        }

        else if (status == BT::NodeStatus::FAILURE)
        {
            publishBTState("IsCoffeFinished", "FAILURE");            
        }        
        return status;

    }

       BT::NodeStatus CoffeMachineROSNode::HasCupOfCoffeBeenRemoved()
    {
        /*std::cout << "HasCupOfCoffeBeenRemoved" << std::endl;
        //CoffeMachineROSNode::publishCmRawImage();
        darknet_ros_msgs::BoundingBox d_bb_cup = CoffeMachineROSNode::find_class(cm_bounding_boxes,"cup");
        darknet_ros_msgs::BoundingBox d_bb_coffeemaker = CoffeMachineROSNode::find_class(cm_bounding_boxes,"coffeemaker");

        if ((d_bb_coffeemaker.probability>0.1) and (not(d_bb_cup.probability>0.6))) 
        {
            std::cout << "The coffe machine has been detected" << std::endl;   
            std::cout << "bounding_box_coffeemaker" << d_bb_coffeemaker << std::endl;        
            CoffeMachineROSNode::plotBoundingBoxesInImage(d_bb_coffeemaker.xmin, d_bb_coffeemaker.ymin, d_bb_coffeemaker.xmax, d_bb_coffeemaker.ymax, d_bb_coffeemaker.id);
            CoffeMachineROSNode::publishBBImage();
            publishBTState("HasCupOfCoffeBeenRemoved", "SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }
        else{
            //CoffeMachineROSNode::plotBoundingBoxesInImage( 0, 0, 0 , 0, -1);
            //CoffeMachineROSNode::publishBBImage();
            publishBTState("HasCupOfCoffeBeenRemoved", "RUNNING");
            return BT::NodeStatus::RUNNING;
        }*/
        status = global.AskForStatus("HasCupOfCoffeBeenRemoved");
        if (status == BT::NodeStatus::SUCCESS)
        {
            publishBTState("HasCupOfCoffeBeenRemoved", "SUCCESS");            
        }
        else if (status == BT::NodeStatus::RUNNING)
        {
            publishBTState("HasCupOfCoffeBeenRemoved", "RUNNING");            
        }

        else if (status == BT::NodeStatus::FAILURE)
        {
            publishBTState("HasCupOfCoffeBeenRemoved", "FAILURE");            
        }        
        return status;

    }
    
       BT::NodeStatus CoffeMachineROSNode::HasHumanAddedCleaningCup()
    {
         //bool image_sent = false;
        std::cout << "HasHumanAddedCleaningCup" << std::endl;
        //CoffeMachineROSNode::publishCmRawImage();
        //if (not (image_sent)){
        auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_rect_color"); //returns a pointer to the message
        CoffeMachineROSNode::copyImage(image);
        pub_yolo_image_raw.publish(image);
        img_encoding_ = image->encoding;//get image encodings
        cv_img_ptr_in_ = cv_bridge::toCvCopy(image, image->encoding);//get image
        //image_sent = true;
        
        
        auto yolo_msg = ros::topic::waitForMessage<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes");
        darknet_ros_msgs::BoundingBox d_bb_cup = CoffeMachineROSNode::find_class(*yolo_msg,"cup");
        darknet_ros_msgs::BoundingBox d_bb_coffeemaker = CoffeMachineROSNode::find_class(*yolo_msg,"coffeemaker");

        if (d_bb_coffeemaker.probability>0.1)
        {
            if (d_bb_cup.probability>0.6)
            {
                    std::cout << "A cup is placed in the coffe machine" << std::endl;   
                    CoffeMachineROSNode::plotBoundingBoxesInImage(d_bb_cup.xmin, d_bb_cup.ymin, d_bb_cup.xmax, d_bb_cup.ymax, d_bb_cup.id,cv_img_ptr_in_);     
                    CoffeMachineROSNode::plotBoundingBoxesInImage(d_bb_coffeemaker.xmin, d_bb_coffeemaker.ymin, d_bb_coffeemaker.xmax, d_bb_coffeemaker.ymax, d_bb_coffeemaker.id,cv_img_ptr_in_);
                    CoffeMachineROSNode::publishBBImage("5.Human_added_cleaning_cup");
                    publishBTState("HasHumanAddedCleaningCup", "SUCCESS");
                    return BT::NodeStatus::SUCCESS;
            
            }
            else{
                    publishBTState("HasHumanAddedCleaningCup", "FAILURE");
                    return BT::NodeStatus::FAILURE;
            }
        }
        else{
            publishBTState("HasHumanAddedCleaningCup", "RUNNING");
            return BT::NodeStatus::RUNNING;
        }


    }

        BT::NodeStatus CoffeMachineROSNode::PlaceCleanCup()
    {
        //_cleancuplaced = true;
        status = global.AskForStatus("PlaceCleanCup");
        if (status == BT::NodeStatus::SUCCESS)
        {
            publishBTState("PlaceCleanCup", "SUCCESS");            
        }
        else if (status == BT::NodeStatus::RUNNING)
        {
            publishBTState("PlaceCleanCup", "RUNNING");            
        }

        else if (status == BT::NodeStatus::FAILURE)
        {
            publishBTState("PlaceCleanCup", "FAILURE");            
        }      
        return status;  
    }

        BT::NodeStatus CoffeMachineROSNode::SwitchOffCoffeMachine()
    {
        //_opened = false;
        status = global.AskForStatus("SwitchOffCoffeMachine");
        if (status == BT::NodeStatus::SUCCESS)
        {
            publishBTState("SwitchOffCoffeMachine", "SUCCESS");            
        }
        else if (status == BT::NodeStatus::RUNNING)
        {
            publishBTState("SwitchOffCoffeMachine", "RUNNING");            
        }

        else if (status == BT::NodeStatus::FAILURE)
        {
            publishBTState("SwitchOffCoffeMachine", "FAILURE");            
        }      
        return status;  

    }


        BT::NodeStatus CoffeMachineROSNode::HasHumanAddedMilk()
    {
         std::cout << "HasHumanAddedMilk" << std::endl;
        auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_rect_color"); 
        CoffeMachineROSNode::copyImage(image);
        pub_yolo_image_raw.publish(image);
        img_encoding_ = image->encoding;//get image encodings
        cv_img_ptr_in_ = cv_bridge::toCvCopy(image, image->encoding);//get image

        auto yolo_msg = ros::topic::waitForMessage<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes");
        darknet_ros_msgs::BoundingBox d_bb_marro_tank = CoffeMachineROSNode::find_class(*yolo_msg,"milk");
        if (d_bb_marro_tank.probability>0.2){
            pub_openpose_image_raw.publish(image);
            auto openpose_msg = ros::topic::waitForMessage<openpose_ros_msgs::OpenPoseHumanList>("/openpose_ros/human_list"); 
            openpose_ros_msgs::PointWithProb op_wrist_join = CoffeMachineROSNode::find_joint(*openpose_msg, 4);
            if (op_wrist_join.prob>0){
                CoffeMachineROSNode::plotBoundingBoxesInImage(d_bb_marro_tank.xmin, d_bb_marro_tank.ymin, d_bb_marro_tank.xmax, d_bb_marro_tank.ymax, d_bb_marro_tank.id,cv_img_ptr_in_);
                CoffeMachineROSNode::plotJointInImage(op_wrist_join.x, op_wrist_join.y, 4,cv_img_ptr_in_);
                CoffeMachineROSNode::publishBBImage("6.Human_has_added_milk");
                publishBTState("HasHumanAddedMilk", "SUCCESS");
                return BT::NodeStatus::SUCCESS;

            }
            else{
                publishBTState("HasHumanAddedMilk", "FAILURE");
                return BT::NodeStatus::FAILURE;
            }
        }
        else{
                publishBTState("HasHumanAddedMilk", "RUNNING");
                return BT::NodeStatus::RUNNING;            
        }
    }


            BT::NodeStatus CoffeMachineROSNode::HasHumanAddedSugar()
    {
      std::cout << "HasHumanAddedMilk" << std::endl;
        auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_rect_color"); 
        CoffeMachineROSNode::copyImage(image);
        pub_yolo_image_raw.publish(image);
        img_encoding_ = image->encoding;//get image encodings
        cv_img_ptr_in_ = cv_bridge::toCvCopy(image, image->encoding);//get image

        auto yolo_msg = ros::topic::waitForMessage<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes");
        darknet_ros_msgs::BoundingBox d_bb_marro_tank = CoffeMachineROSNode::find_class(*yolo_msg,"sugar");
        if (d_bb_marro_tank.probability>0.2){
            pub_openpose_image_raw.publish(image);
            auto openpose_msg = ros::topic::waitForMessage<openpose_ros_msgs::OpenPoseHumanList>("/openpose_ros/human_list"); 
            openpose_ros_msgs::PointWithProb op_wrist_join = CoffeMachineROSNode::find_joint(*openpose_msg, 4);
            if (op_wrist_join.prob>0){
                CoffeMachineROSNode::plotBoundingBoxesInImage(d_bb_marro_tank.xmin, d_bb_marro_tank.ymin, d_bb_marro_tank.xmax, d_bb_marro_tank.ymax, d_bb_marro_tank.id,cv_img_ptr_in_);
                CoffeMachineROSNode::plotJointInImage(op_wrist_join.x, op_wrist_join.y, 4,cv_img_ptr_in_);
                CoffeMachineROSNode::publishBBImage("7.Human_has_added_sugar");
                publishBTState("HasHumanAddedMilk", "SUCCESS");
                return BT::NodeStatus::SUCCESS;

            }
            else{
                publishBTState("HasHumanAddedMilk", "FAILURE");
                return BT::NodeStatus::FAILURE;
            }
        }
        else{
                publishBTState("HasHumanAddedMilk", "RUNNING");
                return BT::NodeStatus::RUNNING;            
        }
  
    }

        BT::NodeStatus CoffeMachineROSNode::IsMilkDesired()
    {
        status = global.AskForStatus("IsMilkDesired");
        if (status == BT::NodeStatus::SUCCESS)
        {
            publishBTState("IsMilkDesired", "SUCCESS");            
        }
        else if (status == BT::NodeStatus::RUNNING)
        {
            publishBTState("IsMilkDesired", "RUNNING");            
        }

        else if (status == BT::NodeStatus::FAILURE)
        {
            publishBTState("IsMilkDesired", "FAILURE");            
        }      
        return status; 
        //std::cout << "[ IsMilkDesired: OK ]" << std::endl;
        //return BT::NodeStatus::SUCCESS;
    }
    
        BT::NodeStatus CoffeMachineROSNode::IsSugarDesired()
    {
        status = global.AskForStatus("IsSugarDesired");
        if (status == BT::NodeStatus::SUCCESS)
        {
            publishBTState("IsSugarDesired", "SUCCESS");            
        }
        else if (status == BT::NodeStatus::RUNNING)
        {
            publishBTState("IsSugarDesired", "RUNNING");            
        }

        else if (status == BT::NodeStatus::FAILURE)
        {
            publishBTState("IsSugarDesired", "FAILURE");            
        }      

        
        return status;         
        //std::cout << "[ IsSugarDesired: OK ]" << std::endl;
        //return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus CoffeMachineROSNode::AddMilkToCoffe()
    {
        //_added = false;
        status = global.AskForStatus("AddMilktoCoffe");
        if (status == BT::NodeStatus::SUCCESS)
        {
            publishBTState("AddMilktoCoffe", "SUCCESS");            
        }
        else if (status == BT::NodeStatus::RUNNING)
        {
            publishBTState("AddMilktoCoffe", "RUNNING");            
        }

        else if (status == BT::NodeStatus::FAILURE)
        {
            publishBTState("AddMilktoCoffe", "FAILURE");            
        }      


        return status;  
        //std::cout << "[ AddMilktoCoffe: OK ]" << std::endl;
        //return BT::NodeStatus::SUCCESS;
    }

    
    BT::NodeStatus CoffeMachineROSNode::AddSugarToCoffe()
    {
        //_added = false;
        status = global.AskForStatus("AddSugarToCoffe");
         if (status == BT::NodeStatus::SUCCESS)
        {
            publishBTState("AddSugarToCoffe", "SUCCESS");            
        }
        else if (status == BT::NodeStatus::RUNNING)
        {
            publishBTState("AddSugarToCoffe", "RUNNING");            
        }

        else if (status == BT::NodeStatus::FAILURE)
        {
            publishBTState("AddSugarToCoffe", "FAILURE");            
        }      
        return status;  
        //std::cout << "[ AddSugartoCoffe: OK ]" << std::endl;
        //return BT::NodeStatus::SUCCESS;
    }

    
    

} //comprovar que el xml està bé.  Ja més endavant entrarem en detall.