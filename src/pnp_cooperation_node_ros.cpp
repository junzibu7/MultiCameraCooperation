//
// Created by hzj on 24-3-29.
//
#include <multi_camera_cooperation/pnp_cooperation_node_ros.h>

PNPCooperationNodeROS::PNPCooperationNodeROS(){
    // empty
}

void PNPCooperationNodeROS::init(ros::NodeHandle &nh, tf::TransformBroadcaster* br, tf::TransformListener* lrA, tf::TransformListener* lrB, tf::TransformListener* lrC, tf::TransformListener* lrD)
{
    br_base_to_coopestimation = br;
    lr_base_to_camA = lrA;
    lr_base_to_camB = lrB;
    lr_base_to_camC = lrC;
    lr_base_to_camD = lrD;

    ROS_INFO("Initial CoopEstimation to Base!");
    t_coopestimation = tf::Vector3(0, 0, 0);
    q_coopestimation = tf::Quaternion(1, 0, 0, 0);
    broadcast_base_to_coopestimation(4);

    sub_camA_to_estimation = nh.subscribe("/camA/single_cam_process_ros/ir_mono/T_cam_to_estimation", 1, &PNPCooperationNodeROS::T_base_to_EstimationfromcamA_callback, this);
    sub_camB_to_estimation = nh.subscribe("/camB/single_cam_process_ros/ir_mono/T_cam_to_estimation", 1, &PNPCooperationNodeROS::T_base_to_EstimationfromcamB_callback, this);
    sub_camC_to_estimation = nh.subscribe("/camC/single_cam_process_ros/ir_mono/T_cam_to_estimation", 1, &PNPCooperationNodeROS::T_base_to_EstimationfromcamC_callback, this);
    sub_camD_to_estimation = nh.subscribe("/camD/single_cam_process_ros/ir_mono/T_cam_to_estimation", 1, &PNPCooperationNodeROS::T_base_to_EstimationfromcamD_callback, this);
    pub_base_to_coopestimation = nh.advertise<geometry_msgs::TransformStamped>("T_base_to_coopestimation", 1);

}

void PNPCooperationNodeROS::T_base_to_EstimationfromcamA_callback(const geometry_msgs::TransformStamped &camA_to_estimation)
{
    //base_to_camA process
    try
    {
        lr_base_to_camA->waitForTransform("base", "camA", ros::Time(0), ros::Duration(1.0));
        lr_base_to_camA->lookupTransform("base", "camA", ros::Time(0), base_to_camA);
        listenercount++;
        camAGoodFlag = true;
        t_base_to_camA = TFVector3ToEigenVector3d(base_to_camA.getOrigin());
        q_base_to_camA = TFQuaternionToEigenQuaterniond(base_to_camA.getRotation());
        R_base_to_camA = q_base_to_camA.toRotationMatrix();
        T_base_to_camA.block<3, 3>(0, 0) = R_base_to_camA;
        T_base_to_camA.block<3, 1>(0, 3) = t_base_to_camA;
        cout<<"T_base_to_camA"<<endl<<T_base_to_camA<<endl;
    }
    catch (tf::TransformException &ex)
    {
        // ROS_ERROR("%s", ex.what());
        camAGoodFlag = false;
        // ros::Duration(1.0).sleep();
    }

    //camA_to_estimation process
    q_camA_to_estimation = TFQuaternionToEigenQuaterniond(tf::Quaternion(camA_to_estimation.transform.rotation.w,  
                          camA_to_estimation.transform.rotation.x,  
                          camA_to_estimation.transform.rotation.y,  
                          camA_to_estimation.transform.rotation.z));  
    t_camA_to_estimation = TFVector3ToEigenVector3d(tf::Vector3(camA_to_estimation.transform.translation.x,  
                       camA_to_estimation.transform.translation.y,  
                       camA_to_estimation.transform.translation.z)); 
    R_camA_to_estimation = q_camA_to_estimation.toRotationMatrix();
    T_camA_to_estimation.block<3, 3>(0, 0) = R_camA_to_estimation;
    T_camA_to_estimation.block<3, 1>(0, 3) = t_camA_to_estimation;
    cout<<"T_camA_to_estimation"<<endl<<T_camA_to_estimation<<endl;

    //base_to_estimation process
    T_base_to_estimationfromcamA = T_base_to_camA * T_camA_to_estimation;
    q_base_to_estimationfromcamA = Eigen::Quaterniond(T_base_to_estimationfromcamA.block<3, 3>(0, 0));
    t_base_to_estimationfromcamA = T_base_to_estimationfromcamA.block<3, 1>(0, 3);
    cout<<"T_base_to_estimationfromcamA"<<endl<<T_base_to_estimationfromcamA<<endl;
    
}

void PNPCooperationNodeROS::T_base_to_EstimationfromcamB_callback(const geometry_msgs::TransformStamped &camB_to_estimation)
{
    //base_to_camB process
    try
    {
        lr_base_to_camB->waitForTransform("base", "camB", ros::Time(0), ros::Duration(1.0));
        lr_base_to_camB->lookupTransform("base", "camB", ros::Time(0), base_to_camB);
        listenercount++;
        camBGoodFlag = true;
        t_base_to_camB = TFVector3ToEigenVector3d(base_to_camB.getOrigin());
        q_base_to_camB = TFQuaternionToEigenQuaterniond(base_to_camB.getRotation());
        R_base_to_camB = q_base_to_camB.toRotationMatrix();
        T_base_to_camB.block<3, 3>(0, 0) = R_base_to_camB;
        T_base_to_camB.block<3, 1>(0, 3) = t_base_to_camB;
    }
    catch (tf::TransformException &ex)
    {
        // ROS_ERROR("%s", ex.what());
        camBGoodFlag = false;
        // ros::Duration(1.0).sleep();
    }

    //camB_to_estimation process
    q_camB_to_estimation = TFQuaternionToEigenQuaterniond(tf::Quaternion(camB_to_estimation.transform.rotation.w,  
                          camB_to_estimation.transform.rotation.x,  
                          camB_to_estimation.transform.rotation.y,  
                          camB_to_estimation.transform.rotation.z));  
    t_camB_to_estimation = TFVector3ToEigenVector3d(tf::Vector3(camB_to_estimation.transform.translation.x,  
                       camB_to_estimation.transform.translation.y,  
                       camB_to_estimation.transform.translation.z)); 
    R_camB_to_estimation = q_camB_to_estimation.toRotationMatrix();
    T_camB_to_estimation.block<3, 3>(0, 0) = R_camB_to_estimation;
    T_camB_to_estimation.block<3, 1>(0, 3) = t_camB_to_estimation;

    //base_to_estimation process
    T_base_to_estimationfromcamB = T_base_to_camB * T_camB_to_estimation;
    q_base_to_estimationfromcamB = Eigen::Quaterniond(T_base_to_estimationfromcamB.block<3, 3>(0, 0));
    t_base_to_estimationfromcamB = T_base_to_estimationfromcamB.block<3, 1>(0, 3);
}

void PNPCooperationNodeROS::T_base_to_EstimationfromcamC_callback(const geometry_msgs::TransformStamped &camC_to_estimation)
{
    //base_to_camC process
    try
    {
        lr_base_to_camC->waitForTransform("base", "camC", ros::Time(0), ros::Duration(1.0));
        lr_base_to_camC->lookupTransform("base", "camC", ros::Time(0), base_to_camC);
        listenercount++;
        camCGoodFlag = true;
        t_base_to_camC = TFVector3ToEigenVector3d(base_to_camC.getOrigin());
        q_base_to_camC = TFQuaternionToEigenQuaterniond(base_to_camC.getRotation());
        R_base_to_camC = q_base_to_camC.toRotationMatrix();
        T_base_to_camC.block<3, 3>(0, 0) = R_base_to_camC;
        T_base_to_camC.block<3, 1>(0, 3) = t_base_to_camC;
    }
    catch (tf::TransformException &ex)
    {
        // ROS_ERROR("%s", ex.what());
        camCGoodFlag = false;
        // ros::Duration(1.0).sleep();
    }

    //camC_to_estimation process
    q_camC_to_estimation = TFQuaternionToEigenQuaterniond(tf::Quaternion(camC_to_estimation.transform.rotation.w,  
                          camC_to_estimation.transform.rotation.x,  
                          camC_to_estimation.transform.rotation.y,  
                          camC_to_estimation.transform.rotation.z));  
    t_camC_to_estimation = TFVector3ToEigenVector3d(tf::Vector3(camC_to_estimation.transform.translation.x,  
                       camC_to_estimation.transform.translation.y,  
                       camC_to_estimation.transform.translation.z)); 
    R_camC_to_estimation = q_camC_to_estimation.toRotationMatrix();
    T_camC_to_estimation.block<3, 3>(0, 0) = R_camC_to_estimation;
    T_camC_to_estimation.block<3, 1>(0, 3) = t_camC_to_estimation;

    //base_to_estimation process
    T_base_to_estimationfromcamC = T_base_to_camC * T_camC_to_estimation;
    q_base_to_estimationfromcamC = Eigen::Quaterniond(T_base_to_estimationfromcamC.block<3, 3>(0, 0));
    t_base_to_estimationfromcamC = T_base_to_estimationfromcamC.block<3, 1>(0, 3);
}

void PNPCooperationNodeROS::T_base_to_EstimationfromcamD_callback(const geometry_msgs::TransformStamped &camD_to_estimation)
{
    //base_to_camD process
    try
    {
        lr_base_to_camD->waitForTransform("base", "camD", ros::Time(0), ros::Duration(1.0));
        lr_base_to_camD->lookupTransform("base", "camD", ros::Time(0), base_to_camD);
        listenercount++;
        camDGoodFlag = true;
        t_base_to_camD = TFVector3ToEigenVector3d(base_to_camD.getOrigin());
        q_base_to_camD = TFQuaternionToEigenQuaterniond(base_to_camD.getRotation());
        R_base_to_camD = q_base_to_camD.toRotationMatrix();
        T_base_to_camD.block<3, 3>(0, 0) = R_base_to_camD;
        T_base_to_camD.block<3, 1>(0, 3) = t_base_to_camD;
    }
    catch (tf::TransformException &ex)
    {
        // ROS_ERROR("%s", ex.what());
        camDGoodFlag = false;
        // ros::Duration(1.0).sleep();
    }

    //camD_to_estimation process
    q_camD_to_estimation = TFQuaternionToEigenQuaterniond(tf::Quaternion(camD_to_estimation.transform.rotation.w,  
                          camD_to_estimation.transform.rotation.x,  
                          camD_to_estimation.transform.rotation.y,  
                          camD_to_estimation.transform.rotation.z));  
    t_camD_to_estimation = TFVector3ToEigenVector3d(tf::Vector3(camD_to_estimation.transform.translation.x,  
                       camD_to_estimation.transform.translation.y,  
                       camD_to_estimation.transform.translation.z)); 
    R_camD_to_estimation = q_camD_to_estimation.toRotationMatrix();
    T_camD_to_estimation.block<3, 3>(0, 0) = R_camD_to_estimation;
    T_camD_to_estimation.block<3, 1>(0, 3) = t_camD_to_estimation;

    //base_to_estimation process
    T_base_to_estimationfromcamD = T_base_to_camD * T_camD_to_estimation;
    q_base_to_estimationfromcamD = Eigen::Quaterniond(T_base_to_estimationfromcamD.block<3, 3>(0, 0));
    t_base_to_estimationfromcamD = T_base_to_estimationfromcamD.block<3, 1>(0, 3);
}


void PNPCooperationNodeROS::broadcast_base_to_coopestimation(int listenercount)
{
    if(listenercount != 0){
        base_to_coopestimation.setOrigin(EigenVector3dToTFVector3(t_base_coopestimation));
        base_to_coopestimation.setRotation(EigenQuaterniondToTFQuaternion(q_base_coopestimation));
        base_to_coopestimation.stamp_ = ros::Time::now();  
        base_to_coopestimation.frame_id_ = "base"; // 父坐标系  
        base_to_coopestimation.child_frame_id_ = "CoopEstimation"; // 子坐标系
        base_to_coopestimation_buf = base_to_coopestimation;
        tf::transformStampedTFToMsg(base_to_coopestimation, EstimationStamped);
        pub_base_to_coopestimation.publish(EstimationStamped);
        br_base_to_coopestimation->sendTransform(EstimationStamped);
    }else{
        base_to_coopestimation_buf.stamp_ = ros::Time::now();  
        tf::transformStampedTFToMsg(base_to_coopestimation_buf, EstimationStamped);
        pub_base_to_coopestimation.publish(EstimationStamped);
        br_base_to_coopestimation->sendTransform(EstimationStamped);
    }           
    
}

void PNPCooperationNodeROS::cam_estimation_fuse(){
    ROS_INFO("CAMERA COOPERATION START!");

    ROS_INFO("Listenercount:%.1i", listenercount);
    if (listenercount != 0)
    {
        t_base_coopestimation = (t_base_to_estimationfromcamA * camAGoodFlag + t_base_to_estimationfromcamB * camBGoodFlag + t_base_to_estimationfromcamC * camCGoodFlag + t_base_to_estimationfromcamD * camDGoodFlag) / listenercount;
        q_base_coopestimation = Eigen::Quaterniond( (q_base_to_estimationfromcamA.w() * camAGoodFlag + q_base_to_estimationfromcamB.w() * camBGoodFlag + q_base_to_estimationfromcamC.w() * camCGoodFlag + q_base_to_estimationfromcamD.w() * camDGoodFlag) / listenercount,
                                                    (q_base_to_estimationfromcamA.x() * camAGoodFlag + q_base_to_estimationfromcamB.x() * camBGoodFlag + q_base_to_estimationfromcamC.x() * camCGoodFlag + q_base_to_estimationfromcamD.x() * camDGoodFlag) / listenercount,
                                                    (q_base_to_estimationfromcamA.y() * camAGoodFlag + q_base_to_estimationfromcamB.y() * camBGoodFlag + q_base_to_estimationfromcamC.y() * camCGoodFlag + q_base_to_estimationfromcamD.y() * camDGoodFlag) / listenercount,
                                                    (q_base_to_estimationfromcamA.z() * camAGoodFlag + q_base_to_estimationfromcamB.z() * camBGoodFlag + q_base_to_estimationfromcamC.z() * camCGoodFlag + q_base_to_estimationfromcamD.z() * camDGoodFlag) / listenercount);    

        broadcast_base_to_coopestimation(listenercount);
        listenercount = 0;
    }
}