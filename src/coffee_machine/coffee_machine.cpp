#include <iostream>
#include "coffee_machine/coffee_machine.h"


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
        param_dist = 100.0;
    }

    void CoffeMachineROSNode::initializeSubscribers()
    {
        ROS_INFO("Initializing Subscribers");
    }

    void CoffeMachineROSNode::initializePublishers()
    {
        ROS_INFO("Initializing Publishers");
        pub_yolo_image_raw = nh.advertise<sensor_msgs::Image>("/coffee_machine/yolo_image_raw", 100);  
        pub_openpose_image_raw = nh.advertise<sensor_msgs::Image>("/coffee_machine/openpose_image_raw", 100);  
        image_transport::ImageTransport img_tp_(nh); 
        image_pub_bb_image_out = img_tp_.advertise("/coffee_machine/image_with_output_data", 100);
        bh_tree = nh.advertise<coffee_machine::State>("/coffee_machine/Feedback", 100);  
        dr_goal = nh.advertise<darknet_ros_msgs::CheckForObjectsActionGoal>("/darknet_ros/check_for_objects/goal", 100);  
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
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("RosImgProcessorNode::image_callback(): cv_bridge exception: %s", e.what());
            return;
        }
    }


    void CoffeMachineROSNode::plotBoundingBoxesInImage(int x_min, int y_min, int x_max, int y_max, int id,cv_bridge::CvImagePtr image)
    {
         cv_bridge::CvImage cv_img_out_;
         cv::Rect_<int> box;
         std::vector<cv::Scalar> color_classes(7);
         color_classes[0] =  cv::Scalar(255,255,255); //milk cv::Scalar(41,70,91); //
         color_classes[1] =  cv::Scalar(128,64,0); //coffe
         color_classes[2] =  cv::Scalar(0,0,0); //coffemaker
         color_classes[3] =  cv::Scalar(255,0,0); //cup
         color_classes[4] =  cv::Scalar(246,210,88); //sugar
         color_classes[5] =  cv::Scalar(90,53,47); //marro_tank
         color_classes[6] =  cv::Scalar(0,128,255); //water_tank

         std::vector<std::string> str_classes(7);
         str_classes[0] = "MILK";
         str_classes[1] = "COFFEE";
         str_classes[2] = "COFFEEMAKER";
         str_classes[3] = "CUP"; //red
         str_classes[4] = "SUGAR";
         str_classes[5] = "MARRO_TANK";
         str_classes[6] = "WATER_TANK";

        ////check if new image is there
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
            cv::rectangle(cv_img_out_.image, box, color_classes[id], 4);
            cv_img_ptr_in_ = nullptr;
            cv::putText(cv_img_out_.image,str_classes[id], cv::Point(x_min,y_min),cv::FONT_HERSHEY_DUPLEX, // Font
                1.2, // Scale. 2.0 = 2x bigger
                color_classes[id], // BGR Color
                1); // Anti-alias (Optional)

        }


    }
    void CoffeMachineROSNode::plotJointInImage(int x, int y, int joint,cv_bridge::CvImagePtr image)
    {
         //std::cout << "holis0" << std::endl;
         cv_bridge::CvImage cv_img_out_;
         std::vector<std::string> name_joint(8);
         name_joint[0] = "NOUSE"; 
         name_joint[1] = "NECK"; 
         name_joint[2] = "RIGHT_SHOULDER"; 
         name_joint[3] = "RIGHT_ELBOW"; 
         name_joint[4] = "RIGHT_WRIST"; 
         name_joint[5] = "LEFT_SHOULDER"; 
         name_joint[6] = "LEFT_ELBOW"; 
         name_joint[7] = "LEFT_WRIST"; 

        std::vector<cv::Scalar> color_classes(8);
         color_classes[0] = cv::Scalar(254,127,156); 
         color_classes[1] = cv::Scalar(202,0,42); 
         color_classes[2] = cv::Scalar(255,105,0); 
         color_classes[3] = cv::Scalar(255,211,0); 
         color_classes[4] = cv::Scalar(255,255,0); 
         color_classes[5] = cv::Scalar(57,255,20); 
         color_classes[6] = cv::Scalar(76,187,23); 
         color_classes[7] = cv::Scalar(11,102,35); 


        if ( image != nullptr )
        {
            // copy the input image to the out one
            cv_img_out_.image = image->image;
        }
        cv::circle(cv_img_out_.image, cv::Point(x, y), 10, color_classes[joint], -1, CV_AA);
        cv::putText(cv_img_out_.image,name_joint[joint], cv::Point(x,y),cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
            1.0, // Scale. 2.0 = 2x bigger
            cv::Scalar(40,40,40), //cv::Scalar(41,70,91),   // BGR Color
            1); // Anti-alias (Optional)    


    }

    void CoffeMachineROSNode::publishBBImage(std::string image_name, cv_bridge::CvImagePtr image)
    {
        cv_bridge::CvImage cv_img_out;
        cv_img_out.image = image->image;
        if( !cv_img_out.image.data ) return;
            cv_img_out.header.seq ++;
            cv_img_out.header.stamp = ros::Time::now();
            cv_img_out.header.frame_id = "camera";
            cv_img_out.encoding =  image->encoding;
            std::string path_ = "/home/julia/tfm/src/coffee_machine/output_img/";   
            cv::imwrite(path_+image_name, cv_bridge::toCvShare(cv_img_out.toImageMsg(), "bgr8")->image) ;
            cv::imshow("image_output", cv_bridge::toCvShare(cv_img_out.toImageMsg(), "bgr8")->image);
            image_pub_bb_image_out.publish(cv_img_out.toImageMsg());
            cv::waitKey(1000); //save the image and go ahead
               
    }

     void CoffeMachineROSNode::publishBTState(std::string NodeType, std::string NodeStatus)
    {
        coffee_machine::State cm_state_msg;
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
            for (int i = 0; i < size; i++)
            {
                if (msg.bounding_boxes[i].Class.compare(dk_class)==0)
                {
                    if (msg.bounding_boxes[i].probability > prob)
                    {
                        prob = msg.bounding_boxes[i].probability;
                        dk_bouding_box = msg.bounding_boxes[i];                      
                    }

                }
            }
            return dk_bouding_box;


        }
        catch(...){
            darknet_ros_msgs::BoundingBox bb_null;        
            bb_null.xmin = 0.0;  
            std::cout << "cannot find a bounding box" << std::endl;
            return bb_null;
        }

    }




    openpose_ros_msgs::PointWithProb CoffeMachineROSNode::find_joint(const openpose_ros_msgs::OpenPoseHumanList& msg, int num_joint)
    {

            openpose_ros_msgs::PointWithProb joint_point_with_prob;
            int size = msg.human_list.size();//sizeof(msg.bounding_boxes)/sizeof(msg.bounding_boxes[0]);
            if (size>0) {
                joint_point_with_prob = msg.human_list[0].body_key_points_with_prob[num_joint];
            }
            return joint_point_with_prob;
 
    }



    

   /* START BT CONDITIONS, ACTIONS*/

   /***************************************************************OpenCoffeMachine Subtree****************************************************************************/


   BT::NodeStatus CoffeMachineROSNode::IsMachineOpen()
    {
        status = global.AskForStatus("IsMachineOpen");

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

    }



    

    BT::NodeStatus CoffeMachineROSNode::IsCleanCupReady(){


        std::cout << "IsCleanCupReady" << std::endl;
        auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_rect_color"); //returns a pointer to the message
        CoffeMachineROSNode::copyImage(image);
        pub_yolo_image_raw.publish(image);
        cv_bridge::CvImagePtr image_in = cv_bridge::toCvCopy(image, image->encoding);//get image
        
        
        auto yolo_msg = ros::topic::waitForMessage<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes");
        darknet_ros_msgs::BoundingBox d_bb_cup = CoffeMachineROSNode::find_class(*yolo_msg,"cup");
        darknet_ros_msgs::BoundingBox d_bb_coffeemaker = CoffeMachineROSNode::find_class(*yolo_msg,"coffeemaker");

        if (d_bb_coffeemaker.probability>0.1)
        {
            if (d_bb_cup.probability>0.2)
            {
                    std::cout << "A cup is placed in the coffee machine" << std::endl;   
                    CoffeMachineROSNode::plotBoundingBoxesInImage(d_bb_cup.xmin, d_bb_cup.ymin, d_bb_cup.xmax, d_bb_cup.ymax, d_bb_cup.id,image_in);     
                    CoffeMachineROSNode::plotBoundingBoxesInImage(d_bb_coffeemaker.xmin, d_bb_coffeemaker.ymin, d_bb_coffeemaker.xmax, d_bb_coffeemaker.ymax, d_bb_coffeemaker.id,image_in);
                    CoffeMachineROSNode::publishBBImage("1.Clean_cup_is_ready.jpg",image_in);
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


    }


    BT::NodeStatus CoffeMachineROSNode::IsWaterTankRemoved(){

        std::cout << "IsWaterTankRemoved" << std::endl;

        unsigned int microsecond = 1000000;
        usleep(5 * microsecond);//sleeps for 3 seconds , giving time to the person to remove the water_tank

        std::cout << "Starting the detection" << std::endl;
 
        auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_rect_color"); 
        CoffeMachineROSNode::copyImage(image);
        pub_yolo_image_raw.publish(image);
        cv_bridge::CvImagePtr image_in = cv_bridge::toCvCopy(image, image->encoding);//get image

        auto yolo_msg = ros::topic::waitForMessage<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes");
        darknet_ros_msgs::BoundingBox d_bb_water_tank = CoffeMachineROSNode::find_class(*yolo_msg,"water_tank");
        if (d_bb_water_tank.probability>0.2){
            pub_openpose_image_raw.publish(image);
            auto openpose_msg = ros::topic::waitForMessage<openpose_ros_msgs::OpenPoseHumanList>("/openpose_ros/human_list"); 
            openpose_ros_msgs::PointWithProb op_wrist_joint_right = CoffeMachineROSNode::find_joint(*openpose_msg, 4);
            openpose_ros_msgs::PointWithProb op_wrist_joint_left = CoffeMachineROSNode::find_joint(*openpose_msg, 7);
            
            if ((op_wrist_joint_right.prob>0)or(op_wrist_joint_left.prob>0)){
                
                CoffeMachineROSNode::plotBoundingBoxesInImage(d_bb_water_tank.xmin, d_bb_water_tank.ymin, d_bb_water_tank.xmax, d_bb_water_tank.ymax, d_bb_water_tank.id,image_in);

                if ((fabs(op_wrist_joint_right.x - 0.5 * (d_bb_water_tank.xmin+d_bb_water_tank.xmax)) < (0.5 *  (d_bb_water_tank.xmax-d_bb_water_tank.xmin) + param_dist)) and
                (fabs(op_wrist_joint_right.y - 0.5 * (d_bb_water_tank.ymin+d_bb_water_tank.ymax)) < (0.5 *  (d_bb_water_tank.ymax-d_bb_water_tank.ymin) + param_dist)))
                {
                    CoffeMachineROSNode::plotJointInImage(op_wrist_joint_right.x, op_wrist_joint_right.y, 4, image_in);
                    std::cout << "Human is handling a water tank with right hand" << std::endl;
                } 
                if ((fabs(op_wrist_joint_left.x - 0.5 * (d_bb_water_tank.xmin+d_bb_water_tank.xmax)) < (0.5 *  (d_bb_water_tank.xmax-d_bb_water_tank.xmin) + param_dist)) and
                (fabs(op_wrist_joint_left.y - 0.5 * (d_bb_water_tank.ymin+d_bb_water_tank.ymax)) < (0.5 *  (d_bb_water_tank.ymax-d_bb_water_tank.ymin) + param_dist)))
                {
                    CoffeMachineROSNode::plotJointInImage(op_wrist_joint_left.x, op_wrist_joint_left.y, 7, image_in);
                    std::cout << "Human is handling a water tank with left hand" << std::endl;
                }  

                CoffeMachineROSNode::publishBBImage("2.Human_handling_water_tank.jpg",image_in);
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
        std::cout << "IsWaterTankPlacedInCoffeMachine" << std::endl;
        unsigned int microsecond = 1000000;
        usleep(5 * microsecond);//sleeps for 5 seconds , giving time to the person to place the water_tank in the coffe_machine

        std::cout << "Starting the detection" << std::endl;
        auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_rect_color"); 
        //CoffeMachineROSNode::copyImage(image);
        //cv_bridge::CvImagePtr image_in = cv_bridge::toCvCopy(image, image->encoding);//get imageolo

        darknet_ros_msgs::CheckForObjectsActionGoal CheckForobjectsActionGoal;
        CheckForobjectsActionGoal.goal_id.id = "6";
        CheckForobjectsActionGoal.goal.id = 6;
        CheckForobjectsActionGoal.goal.image = *image;
        dr_goal.publish(CheckForobjectsActionGoal);

        
        auto actionresult = ros::topic::waitForMessage<darknet_ros_msgs::CheckForObjectsActionResult>("/darknet_ros/check_for_objects/result",ros::Duration(10)); 
        if(actionresult != NULL){
            darknet_ros_msgs::CheckForObjectsActionResult result = *actionresult;
            darknet_ros_msgs::BoundingBox d_bb_water_tank = CoffeMachineROSNode::find_class(result.result.bounding_boxes,"water_tank");
            if ((d_bb_water_tank.probability>0.2)){
                    publishBTState("IsWaterTankPlacedInCoffeMachine", "RUNNING");
                    return BT::NodeStatus::RUNNING;  
            }
            else{
                    std::cout << "No water tank found in the coffe machine" << std::endl;
                    publishBTState("IsWaterTankPlacedInCoffeMachine", "SUCCESS");
                    return BT::NodeStatus::SUCCESS;  
            }
        }
        else{
                    std::cout << "No water tank found in the coffe machine" << std::endl;
                    publishBTState("IsWaterTankPlacedInCoffeMachine", "SUCCESS");
                    return BT::NodeStatus::SUCCESS;  
            }
            
        

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

        unsigned int microsecond = 1000000;
        usleep(5 * microsecond);//sleeps for 3 seconds , giving time to the person to remove the marro tank


        std::cout << "Starting the detection" << std::endl;
        auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_rect_color"); 
        CoffeMachineROSNode::copyImage(image);
        pub_yolo_image_raw.publish(image);
        cv_bridge::CvImagePtr image_in = cv_bridge::toCvCopy(image, image->encoding);//get image


        auto yolo_msg = ros::topic::waitForMessage<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes");
        darknet_ros_msgs::BoundingBox d_bb_marro_tank = CoffeMachineROSNode::find_class(*yolo_msg,"marro_tank");
        if (d_bb_marro_tank.probability>0.2){
            pub_openpose_image_raw.publish(image);
            auto openpose_msg = ros::topic::waitForMessage<openpose_ros_msgs::OpenPoseHumanList>("/openpose_ros/human_list"); 
            openpose_ros_msgs::PointWithProb op_wrist_joint_right = CoffeMachineROSNode::find_joint(*openpose_msg, 4);
            openpose_ros_msgs::PointWithProb op_wrist_joint_left = CoffeMachineROSNode::find_joint(*openpose_msg, 7);
            if ((op_wrist_joint_right.prob>0)or(op_wrist_joint_left.prob>0)){
                CoffeMachineROSNode::plotBoundingBoxesInImage(d_bb_marro_tank.xmin, d_bb_marro_tank.ymin, d_bb_marro_tank.xmax, d_bb_marro_tank.ymax, d_bb_marro_tank.id,image_in);

                if ((fabs(op_wrist_joint_right.x - 0.5 * (d_bb_marro_tank.xmin+d_bb_marro_tank.xmax)) < (0.5 *  (d_bb_marro_tank.xmax-d_bb_marro_tank.xmin) + param_dist))and
                (fabs(op_wrist_joint_right.y - 0.5 * (d_bb_marro_tank.ymin+d_bb_marro_tank.ymax)) < (0.5 *  (d_bb_marro_tank.ymax-d_bb_marro_tank.ymin) + param_dist)))
                {
                    CoffeMachineROSNode::plotJointInImage(op_wrist_joint_right.x, op_wrist_joint_right.y, 4, image_in);
                    std::cout << "Human is handling a marro tank with right hand" << std::endl;
                } 
                if ((fabs(op_wrist_joint_left.x - 0.5 * (d_bb_marro_tank.xmin+d_bb_marro_tank.xmax)) < (0.5 *  (d_bb_marro_tank.xmax-d_bb_marro_tank.xmin) + param_dist))and
                (fabs(op_wrist_joint_left.y - 0.5 * (d_bb_marro_tank.ymin+d_bb_marro_tank.ymax)) < (0.5 *  (d_bb_marro_tank.ymax-d_bb_marro_tank.ymin) + param_dist)))
                {
                    CoffeMachineROSNode::plotJointInImage(op_wrist_joint_left.x, op_wrist_joint_left.y, 7, image_in);
                    std::cout << "Human is handling a marro tank with left hand" << std::endl;
                }  

                CoffeMachineROSNode::publishBBImage("3.Human_handling_marro_tank.jpg",image_in);
                publishBTState("IsMarroTankRemoved", "SUCCESS");
                return BT::NodeStatus::SUCCESS;


            }
            else{
                publishBTState("IsMarroTankRemoved", "FAILURE");
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
        std::cout << "IsMarroTankPlacedInCoffeMachine" << std::endl;
        unsigned int microsecond = 1000000;
        usleep(5 * microsecond);//sleeps for 5 seconds , giving time to the person to place the marro_tank in the coffe_machine
        std::cout << "Starting the detection" << std::endl;
        
        auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_rect_color"); 
        //CoffeMachineROSNode::copyImage(image);
        //cv_bridge::CvImagePtr image_in = cv_bridge::toCvCopy(image, image->encoding);//get imageolo

        darknet_ros_msgs::CheckForObjectsActionGoal CheckForobjectsActionGoal;
        CheckForobjectsActionGoal.goal_id.id = "5";
        CheckForobjectsActionGoal.goal.id = 5;
        CheckForobjectsActionGoal.goal.image = *image;
        dr_goal.publish(CheckForobjectsActionGoal);

        //std::cout << "holis" << std::endl;
        auto actionresult = ros::topic::waitForMessage<darknet_ros_msgs::CheckForObjectsActionResult>("/darknet_ros/check_for_objects/result",ros::Duration(10)); 
        if(actionresult != NULL){
    


            darknet_ros_msgs::CheckForObjectsActionResult result = *actionresult;
            darknet_ros_msgs::BoundingBox d_bb_marro_tank = CoffeMachineROSNode::find_class(result.result.bounding_boxes,"marro_tank");
            if (d_bb_marro_tank.probability>0.2){
                    publishBTState("IsMarroTankPlacedInCoffeMachine", "RUNNING");
                    return BT::NodeStatus::RUNNING;  
            }
            else{
                    std::cout << "No marro tank found in the coffe machine" << std::endl;
                    publishBTState("IsMarroTankPlacedInCoffeMachine", "SUCCESS");
                    return BT::NodeStatus::SUCCESS;  
            }
        }
        else{
                    std::cout << "No marro tank found in the coffe machine" << std::endl;
                    publishBTState("IsMarroTankPlacedInCoffeMachine", "SUCCESS");
                    return BT::NodeStatus::SUCCESS;  
            }
            
    }

        /*******************************************PutCoffeCup Subtree******************************************************/


        BT::NodeStatus CoffeMachineROSNode::IsCoffeCupReady()
    {
        std::cout << "IsCoffeCupReady" << std::endl;
        unsigned int microsecond = 1000000;
        usleep(5 * microsecond);//sleeps for 3 seconds

        std::cout << "Starting the detection" << std::endl;
        auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_rect_color"); //returns a pointer to the message
        CoffeMachineROSNode::copyImage(image);
        pub_yolo_image_raw.publish(image);
        cv_bridge::CvImagePtr image_in = cv_bridge::toCvCopy(image, image->encoding);//get image
        
        
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
                    CoffeMachineROSNode::publishBBImage("4.Coffe_cup_is_ready.jpg",image_in);
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

        BT::NodeStatus CoffeMachineROSNode::PlaceCoffeCup() //estaria bé posar-ho
    {

        std::cout << "PlaceCoffeCup" << std::endl;
        unsigned int microsecond = 1000000;
        usleep(5 * microsecond);//sleeps for 3 seconds

        std::cout << "Starting the detection" << std::endl;
        auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_rect_color"); //returns a pointer to the message
        CoffeMachineROSNode::copyImage(image);
        pub_yolo_image_raw.publish(image);
        cv_bridge::CvImagePtr image_in = cv_bridge::toCvCopy(image, image->encoding);//get image
        
        
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
                    CoffeMachineROSNode::publishBBImage("5.Coffe_cup_is_placed.jpg",image_in);
                    publishBTState("CoffeCupPlaced", "SUCCESS");
                    return BT::NodeStatus::SUCCESS;
            
            }
            else{
                    publishBTState("CoffeCupPlaced", "FAILURE");
                    return BT::NodeStatus::FAILURE;
            }
        }
        else{
            publishBTState("CoffeCupPlaced", "RUNNING");
            return BT::NodeStatus::RUNNING;

        }


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
    
        unsigned int microsecond = 1000000;
        usleep(5 * microsecond);//sleeps for 3 seconds 
        std::cout << "IsWaterTankPlacedInCoffeMachine" << std::endl;
        auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_rect_color"); 
        //CoffeMachineROSNode::copyImage(image);
        //cv_bridge::CvImagePtr image_in = cv_bridge::toCvCopy(image, image->encoding);//get imageo

        darknet_ros_msgs::CheckForObjectsActionGoal CheckForobjectsActionGoal;
        CheckForobjectsActionGoal.goal_id.id = "1";
        CheckForobjectsActionGoal.goal.id = 1;
        CheckForobjectsActionGoal.goal.image = *image;
        dr_goal.publish(CheckForobjectsActionGoal);

        
        auto actionresult = ros::topic::waitForMessage<darknet_ros_msgs::CheckForObjectsActionResult>("/darknet_ros/check_for_objects/result",ros::Duration(10)); 
        if(actionresult != NULL){
            darknet_ros_msgs::CheckForObjectsActionResult result = *actionresult;
        
            if (result.result.bounding_boxes.bounding_boxes[0].Class == "coffee"){
                    publishBTState("HasCupOfCoffeBeenRemoved", "RUNNING");
                    return BT::NodeStatus::RUNNING;  
            }
            else{
                    std::cout << "No cup of coffe found in the coffe machine" << std::endl;
                    publishBTState("HasCupOfCoffeBeenRemoved", "SUCCESS");
                    return BT::NodeStatus::SUCCESS;  
            }
        }
        else{
                    std::cout << "No cup of coffe found in the coffe machine" << std::endl;
                    publishBTState("HasCupOfCoffeBeenRemoved", "SUCCESS");
                    return BT::NodeStatus::SUCCESS;  
            }

    }
    
       BT::NodeStatus CoffeMachineROSNode::HasHumanAddedCleaningCup()
    {
        std::cout << "HasHumanAddedCleaningCup" << std::endl;
        unsigned int microsecond = 1000000;
        usleep(5 * microsecond);//sleeps for 3 seconds 

        std::cout << "Starting the detection" << std::endl;
        auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_rect_color"); //returns a pointer to the message
        CoffeMachineROSNode::copyImage(image);
        pub_yolo_image_raw.publish(image);
        cv_bridge::CvImagePtr image_in = cv_bridge::toCvCopy(image, image->encoding);//get image
        
        
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
                    CoffeMachineROSNode::publishBBImage("6.Clean_cup_has_been_added.jpg",image_in);
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
      
        unsigned int microsecond = 1000000;
        usleep(7 * microsecond);//sleeps for 3 seconds 

        std::cout << "Starting the detection" << std::endl;

        auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_rect_color"); 
        CoffeMachineROSNode::copyImage(image);
        pub_yolo_image_raw.publish(image);

        cv_bridge::CvImagePtr image_in = cv_bridge::toCvCopy(image, image->encoding);//get image

        auto yolo_msg = ros::topic::waitForMessage<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes");
        darknet_ros_msgs::BoundingBox d_bb_milk = CoffeMachineROSNode::find_class(*yolo_msg,"milk");
        if (d_bb_milk.probability>0.2){
            pub_openpose_image_raw.publish(image);
            auto openpose_msg = ros::topic::waitForMessage<openpose_ros_msgs::OpenPoseHumanList>("/openpose_ros/human_list"); 
            openpose_ros_msgs::PointWithProb op_wrist_joint_right = CoffeMachineROSNode::find_joint(*openpose_msg, 4);
            openpose_ros_msgs::PointWithProb op_wrist_joint_left = CoffeMachineROSNode::find_joint(*openpose_msg, 7);
            if ((op_wrist_joint_right.prob>0)and(op_wrist_joint_left.prob>0)){
                CoffeMachineROSNode::plotBoundingBoxesInImage(d_bb_milk.xmin, d_bb_milk.ymin, d_bb_milk.xmax, d_bb_milk.ymax, d_bb_milk.id,image_in);
 
                if ((fabs(op_wrist_joint_right.x - 0.5 * (d_bb_milk.xmin+d_bb_milk.xmax)) < (0.5 *  (d_bb_milk.xmax-d_bb_milk.xmin) + param_dist))and
                (fabs(op_wrist_joint_right.y - 0.5 * (d_bb_milk.ymin+d_bb_milk.ymax)) < (0.5 *  (d_bb_milk.ymax-d_bb_milk.ymin) + param_dist)))
                {
                    CoffeMachineROSNode::plotJointInImage(op_wrist_joint_right.x, op_wrist_joint_right.y, 4, image_in);
                    std::cout << "Human is handling milk with right hand" << std::endl;
                } 
                if ((fabs(op_wrist_joint_left.x - 0.5 * (d_bb_milk.xmin+d_bb_milk.xmax)) < (0.5 *  (d_bb_milk.xmax-d_bb_milk.xmin) + param_dist))and
                (fabs(op_wrist_joint_left.y - 0.5 * (d_bb_milk.ymin+d_bb_milk.ymax)) < (0.5 *  (d_bb_milk.ymax-d_bb_milk.ymin) + param_dist)))
                {
                    CoffeMachineROSNode::plotJointInImage(op_wrist_joint_left.x, op_wrist_joint_left.y, 7, image_in);
                    std::cout << "Human is handling milk with left hand" << std::endl;
                }  

                CoffeMachineROSNode::publishBBImage("7.Human_adding_milk.jpg",image_in);
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
        std::cout << "HasHumanAddedSugar" << std::endl;
         unsigned int microsecond = 1000000;
        usleep(7 * microsecond);//sleeps for 3 seconds 

        std::cout << "Starting the detection" << std::endl;

        auto image = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_rect_color"); 
        CoffeMachineROSNode::copyImage(image);
        pub_yolo_image_raw.publish(image);

        cv_bridge::CvImagePtr image_in = cv_bridge::toCvCopy(image, image->encoding);//get image

        auto yolo_msg = ros::topic::waitForMessage<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes");
        darknet_ros_msgs::BoundingBox d_bb_sugar = CoffeMachineROSNode::find_class(*yolo_msg,"sugar");
        if (d_bb_sugar.probability>0.2){
            pub_openpose_image_raw.publish(image);
            auto openpose_msg = ros::topic::waitForMessage<openpose_ros_msgs::OpenPoseHumanList>("/openpose_ros/human_list"); 
            openpose_ros_msgs::PointWithProb op_wrist_joint_right = CoffeMachineROSNode::find_joint(*openpose_msg, 4);
            openpose_ros_msgs::PointWithProb op_wrist_joint_left = CoffeMachineROSNode::find_joint(*openpose_msg, 7);
            if ((op_wrist_joint_right.prob>0)and(op_wrist_joint_left.prob>0)){
                CoffeMachineROSNode::plotBoundingBoxesInImage(d_bb_sugar.xmin, d_bb_sugar.ymin, d_bb_sugar.xmax, d_bb_sugar.ymax, d_bb_sugar.id,image_in);

                if ((fabs(op_wrist_joint_right.x - 0.5 * (d_bb_sugar.xmin+d_bb_sugar.xmax)) < (0.5 *  (d_bb_sugar.xmax-d_bb_sugar.xmin) + param_dist))and
                (fabs(op_wrist_joint_right.y - 0.5 * (d_bb_sugar.ymin+d_bb_sugar.ymax)) < (0.5 *  (d_bb_sugar.ymax-d_bb_sugar.ymin) + param_dist)))
                {
                    CoffeMachineROSNode::plotJointInImage(op_wrist_joint_right.x, op_wrist_joint_right.y, 4, image_in);
                    std::cout << "Human is handling sugar with right hand" << std::endl;
                } 
                if ((fabs(op_wrist_joint_left.x - 0.5 * (d_bb_sugar.xmin+d_bb_sugar.xmax)) < (0.5 *  (d_bb_sugar.xmax-d_bb_sugar.xmin) + param_dist))and
                (fabs(op_wrist_joint_left.y - 0.5 * (d_bb_sugar.ymin+d_bb_sugar.ymax)) < (0.5 *  (d_bb_sugar.ymax-d_bb_sugar.ymin) + param_dist)))
                {
                    CoffeMachineROSNode::plotJointInImage(op_wrist_joint_left.x, op_wrist_joint_left.y, 7, image_in);
                    std::cout << "Human is handling sugar with left hand" << std::endl;
                }  

                CoffeMachineROSNode::publishBBImage("8.Human_adding_sugar.jpg",image_in);
                publishBTState("HasHumanAddedSugar", "SUCCESS");
                return BT::NodeStatus::SUCCESS;


            }
            else{
                publishBTState("HasHumanAddedSugar", "FAILURE");
                return BT::NodeStatus::FAILURE;
            }
        }
        else{
                publishBTState("HasHumanAddedSugar", "RUNNING");
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