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


    uav_vicon_pos_sub = nh.subscribe("/csj01/csj01/mocap/pos", 1, &PNPCooperationNodeROS::Vicon_uav_pos, this);
    vicon_frame_sub = nh.subscribe("/zwbframe/zwbframe/mocap/pos", 1, &PNPCooperationNodeROS::Vicon_pnp_test_frame, this);
    pub_base_to_coopestimation = nh.advertise<geometry_msgs::TransformStamped>("T_base_to_coopestimation", 1);
    pub_base_to_coopestimation_from_camA = nh.advertise<geometry_msgs::TransformStamped>("T_base_to_coopestimation_from_camA", 1);
    pub_base_to_coopestimation_from_camB = nh.advertise<geometry_msgs::TransformStamped>("T_base_to_coopestimation_from_camB", 1);
    pub_base_to_coopestimation_from_camC = nh.advertise<geometry_msgs::TransformStamped>("T_base_to_coopestimation_from_camC", 1);
    pub_base_to_coopestimation_from_camD = nh.advertise<geometry_msgs::TransformStamped>("T_base_to_coopestimation_from_camD", 1);


    pub_error_camA = nh.advertise<geometry_msgs::TransformStamped>("error_camA",1);
    pub_error_camD = nh.advertise<geometry_msgs::TransformStamped>("error_camB",1);

}

void PNPCooperationNodeROS::toRotationMatrix(float x, float y,float z,float w,Eigen::Matrix3Xd &R){
     R << 1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w,
             2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w,
             2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y;
}

void PNPCooperationNodeROS::Vicon_pnp_test_frame(const geometry_msgs::PoseStamped::ConstPtr &msg){
    frame_vicon_pos = uav_vicon_pos-Eigen::Vector3d((msg->pose.position.x), (msg->pose.position.y), (msg->pose.position.z));
    ROS_INFO("csj01_pos:: x%.3f,y%.3f,z%.3f",msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    // ROS_INFO("frame_vicon_pos:x%.3f, y%.3f, z%.3f", frame_vicon_pos(0), frame_vicon_pos(1), frame_vicon_pos(2));
    
    
    toRotationMatrix(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w,world_to_cameraA_R);

    cameraA_to_world_R = world_to_cameraA_R.transpose();
    
}

void PNPCooperationNodeROS::Vicon_uav_pos(const geometry_msgs::PoseStamped::ConstPtr &msg){
    uav_vicon_pos = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    ROS_INFO("uav_vicon_pos:x%.3f, y%.3f, z%.3f", uav_vicon_pos(0), uav_vicon_pos(1), uav_vicon_pos(2));
}

void PNPCooperationNodeROS::T_base_to_EstimationfromcamA_callback(const geometry_msgs::TransformStamped &camA_to_estimation)
{
    //base_to_camA process
    try
    {
        lr_base_to_camA->waitForTransform("base", "camA", ros::Time(0), ros::Duration(1.0));
        lr_base_to_camA->lookupTransform("base", "camA", ros::Time(0), base_to_camA);
        listenercount++;
        ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        camAGoodFlag = true;
        t_base_to_camA = TFVector3ToEigenVector3d(base_to_camA.getOrigin());
        q_base_to_camA = TFQuaternionToEigenQuaterniond(base_to_camA.getRotation());
        R_base_to_camA = q_base_to_camA.toRotationMatrix();
        T_base_to_camA.block<3, 3>(0, 0) = R_base_to_camA;
        T_base_to_camA.block<3, 1>(0, 3) = t_base_to_camA;
        //cout<<"T_base_to_camA"<<endl<<T_base_to_camA<<endl;
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
    // t_camA_to_estimation = TFVector3ToEigenVector3d(tf::Vector3(camA_to_estimation.transform.translation.x,  
    //                    camA_to_estimation.transform.translation.y,  
    //                    camA_to_estimation.transform.translation.z)); 



    t_camA_to_estimation = TFVector3ToEigenVector3d(tf::Vector3(camA_to_estimation.transform.translation.x,  
                       camA_to_estimation.transform.translation.y,  
                       camA_to_estimation.transform.translation.z)); 
    
    R_camA_to_estimation = q_camA_to_estimation.toRotationMatrix();
    T_camA_to_estimation.block<3, 3>(0, 0) = R_camA_to_estimation;
    T_camA_to_estimation.block<3, 1>(0, 3) = t_camA_to_estimation;
    //cout<<"T_camA_to_estimation"<<endl<<T_camA_to_estimation<<endl;

    //base_to_estimation process
    T_base_to_estimationfromcamA = T_base_to_camA * T_camA_to_estimation;
    q_base_to_estimationfromcamA = Eigen::Quaterniond(T_base_to_estimationfromcamA.block<3, 3>(0, 0));
    t_base_to_estimationfromcamA = T_base_to_estimationfromcamA.block<3, 1>(0, 3);

    // delta_x, delta_y, delta_z reprensents the displcaement of the base from uwb anchor00
    double delta_xA = 10;
    double delta_yA = 10;
    double delta_zA = 10;
    //cout<<"T_base_to_estimationfromcamA"<<endl<<T_base_to_estimationfromcamA<<endl;

    ros::Rate r(10000);
    if (camAGoodFlag)
    {   
        baseA_to_coopestimation.setOrigin(EigenVector3dToTFVector3(t_base_to_estimationfromcamA));
        baseA_to_coopestimation.setRotation(EigenQuaterniondToTFQuaternion(q_base_to_estimationfromcamA));
        baseA_to_coopestimation.stamp_ = ros::Time::now();  

        baseA_to_coopestimation.frame_id_ = "Dbase"; // 父坐标系  
        baseA_to_coopestimation.child_frame_id_ = "CoopEstimation"; // 子坐标系
        tf::transformStampedTFToMsg(baseA_to_coopestimation, EstimationStampedfromcamA);
        
        // pub_base_to_coopestimation.publish(EstimationStampedfromcamD);

        pub_base_to_coopestimation_from_camA.publish(EstimationStampedfromcamA);
        delta_xA = uav_vicon_pos(0) - 0.997- t_base_to_estimationfromcamA(0);
        delta_yA = uav_vicon_pos(1) + 1.311- t_base_to_estimationfromcamA(1);
        delta_zA = uav_vicon_pos(2) - 0.372-t_base_to_estimationfromcamA(2);

        error_camA_vector.transform.translation.x = delta_xA;
        error_camA_vector.transform.translation.y = delta_yA;
        error_camA_vector.transform.translation.z = delta_zA;

        // pub_error_camA.publish(error_camA_vector);
        
        //ROS_INFO("delta_xA:%.3f, delta_yA:%.3f, delta_zA:%.3f", abs(delta_xA), abs(delta_yA), abs(delta_zA));
        //r.sleep();

    }
    else
    {
        ROS_INFO("camA_to_estimation_from uwb00 is not published!");
    }
    pub_error_camA.publish(error_camA_vector);
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

    double tamp = camB_to_estimation.header.stamp.toSec();
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

    double delta_xB = 0;
    double delta_yB = 0;
    double delta_zB = 0;
    t_base_to_estimationfromcamB(0) = t_base_to_estimationfromcamB(0) + delta_xB;
    t_base_to_estimationfromcamB(1) = t_base_to_estimationfromcamB(1) + delta_yB;
    t_base_to_estimationfromcamB(2) = t_base_to_estimationfromcamB(2) + delta_zB;
    cout<<"T_base_to_estimationfromcamB"<<endl<<T_base_to_estimationfromcamB<<endl;
    if (camBGoodFlag)
    {   
        baseB_to_coopestimation.setOrigin(EigenVector3dToTFVector3(t_base_to_estimationfromcamB));
        baseB_to_coopestimation.setRotation(EigenQuaterniondToTFQuaternion(q_base_to_estimationfromcamB));
        baseB_to_coopestimation.stamp_ = ros::Time::now();  
        baseB_to_coopestimation.frame_id_ = "Dbase"; // 父坐标系  
        baseB_to_coopestimation.child_frame_id_ = "CoopEstimation"; // 子坐标系
        tf::transformStampedTFToMsg(baseB_to_coopestimation, EstimationStampedfromcamB);
        
        // pub_base_to_coopestimation.publish(EstimationStampedfromcamD);

        pub_base_to_coopestimation_from_camB.publish(EstimationStampedfromcamB);
        
        
    }
    else
    {
        ROS_INFO("camB_to_estimation_from uwb00 is not published!");
    }
    
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

    double delta_xC = 0;
    double delta_yC = 0;
    double delta_zC = 0;
    t_base_to_estimationfromcamB(0) = t_base_to_estimationfromcamB(0) + delta_xC;
    t_base_to_estimationfromcamB(1) = t_base_to_estimationfromcamB(1) + delta_yC;
    t_base_to_estimationfromcamB(2) = t_base_to_estimationfromcamB(2) + delta_zC;
    cout<<"T_base_to_estimationfromcamC"<<endl<<T_base_to_estimationfromcamC<<endl;
    if (camCGoodFlag)
    {   
        baseC_to_coopestimation.setOrigin(EigenVector3dToTFVector3(t_base_to_estimationfromcamC));
        baseC_to_coopestimation.setRotation(EigenQuaterniondToTFQuaternion(q_base_to_estimationfromcamC));
        baseC_to_coopestimation.stamp_ = ros::Time::now();  
        baseC_to_coopestimation.frame_id_ = "Dbase"; // 父坐标系  
        baseC_to_coopestimation.child_frame_id_ = "CoopEstimation"; // 子坐标系
        tf::transformStampedTFToMsg(baseC_to_coopestimation, EstimationStampedfromcamC);
        
        // pub_base_to_coopestimation.publish(EstimationStampedfromcamD);

        pub_base_to_coopestimation_from_camC.publish(EstimationStampedfromcamC);
    }
    else
    {
        ROS_INFO("camC_to_estimation_from uwb00 is not published!");
    }
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

    double delta_xD = 10;
    double delta_yD = 10;
    double delta_zD = 10;
    t_base_to_estimationfromcamD(0) = t_base_to_estimationfromcamD(0) ;
    t_base_to_estimationfromcamD(1) = t_base_to_estimationfromcamD(1) ;
    t_base_to_estimationfromcamD(2) = t_base_to_estimationfromcamD(2) ;
    ros::Rate r(10000);
    //cout<<"T_base_to_estimationfromcamD"<<endl<<T_base_to_estimationfromcamD<<endl;
    if (camDGoodFlag)
    {   
        baseD_to_coopestimation.setOrigin(EigenVector3dToTFVector3(t_base_to_estimationfromcamD));
        baseD_to_coopestimation.setRotation(EigenQuaterniondToTFQuaternion(q_base_to_estimationfromcamD));
        baseD_to_coopestimation.stamp_ = ros::Time::now();  
        baseD_to_coopestimation.frame_id_ = "Dbase"; // 父坐标系  
        baseD_to_coopestimation.child_frame_id_ = "CoopEstimation"; // 子坐标系
        tf::transformStampedTFToMsg(baseD_to_coopestimation, EstimationStampedfromcamD);
        
        // pub_base_to_coopestimation.publish(EstimationStampedfromcamD);

        pub_base_to_coopestimation_from_camD.publish(EstimationStampedfromcamD);
        delta_xD = uav_vicon_pos(0) - 0.997- t_base_to_estimationfromcamD(0);
        delta_yD = uav_vicon_pos(1) + 1.311- t_base_to_estimationfromcamD(1);
        delta_zD = uav_vicon_pos(2) - 0.372- t_base_to_estimationfromcamD(2);

        error_camD_vector.transform.translation.x = delta_xD;
        error_camD_vector.transform.translation.y = delta_yD;
        error_camD_vector.transform.translation.z = delta_zD;

        //pub_error_camD.publish(error_camD_vector);
        //r.sleep();
    }
    else
    {
        ROS_INFO("camD_to_estimation_from uwb00 is not published!");
    }

    pub_error_camD.publish(error_camD_vector);
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
    ros::Rate loop_rate(100);
    ROS_INFO("Listenercount:%.1i", listenercount);
    if (listenercount != 0)
    {
        t_base_coopestimation = (t_base_to_estimationfromcamA * camAGoodFlag + t_base_to_estimationfromcamB * camBGoodFlag + t_base_to_estimationfromcamC * camCGoodFlag + t_base_to_estimationfromcamD * camDGoodFlag) / listenercount;
        // t_base_coopestimation = t_base_to_estimationfromcamA;
        ROS_INFO("t_base_coopestimation_from_camA:x%.3f, y%.3f, z%.3f", t_base_coopestimation(0), t_base_coopestimation(1), t_base_coopestimation(2));

        



        // vicon_pnp_error = t_base_coopestimation_inworld - frame_vicon_pos;
        // ROS_INFO("vicon_pnp_error:delta_x%.3f, delta_y%.3f, delta_z%.3f", abs(vicon_pnp_error(0)), abs(vicon_pnp_error(1)), abs(vicon_pnp_error(2)));

        // how to sleep for 0.1s
        // loop_rate.sleep();


        q_base_coopestimation = Eigen::Quaterniond( (q_base_to_estimationfromcamA.w() * camAGoodFlag + q_base_to_estimationfromcamB.w() * camBGoodFlag + q_base_to_estimationfromcamC.w() * camCGoodFlag + q_base_to_estimationfromcamD.w() * camDGoodFlag) / listenercount,
                                                    (q_base_to_estimationfromcamA.x() * camAGoodFlag + q_base_to_estimationfromcamB.x() * camBGoodFlag + q_base_to_estimationfromcamC.x() * camCGoodFlag + q_base_to_estimationfromcamD.x() * camDGoodFlag) / listenercount,
                                                    (q_base_to_estimationfromcamA.y() * camAGoodFlag + q_base_to_estimationfromcamB.y() * camBGoodFlag + q_base_to_estimationfromcamC.y() * camCGoodFlag + q_base_to_estimationfromcamD.y() * camDGoodFlag) / listenercount,
                                                    (q_base_to_estimationfromcamA.z() * camAGoodFlag + q_base_to_estimationfromcamB.z() * camBGoodFlag + q_base_to_estimationfromcamC.z() * camCGoodFlag + q_base_to_estimationfromcamD.z() * camDGoodFlag) / listenercount);    

        broadcast_base_to_coopestimation(listenercount);
        listenercount = 0;
    }
}