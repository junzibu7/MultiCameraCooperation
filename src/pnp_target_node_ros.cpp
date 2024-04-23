//
// Created by hzj on 24-2-28.
//
#include "multi_camera_cooperation/pnp_target_node_ros.h"

using namespace std;

PnPTargetNodeROS::PnPTargetNodeROS(){
    
}

void PnPTargetNodeROS::init(ros::NodeHandle &nh, tf::TransformBroadcaster* br){

//============== Read ros parameter =====================//
    std::string uav_config_file;
    int drone_id;
    br0 = br;

    nh.param<std::string>("cam", cam, "camA"); 
    nh.param<double>("PnP_time_delay", PnP_time_delay, 0.0);//PnP检测中，由相机捕获、程序处理、滤波等带来的时间延迟
    nh.param<int>("drone_id", drone_id, 0);
    nh.param<std::string>("uav_config_file", uav_config_file, "default");
    nh.param<std::string>("write_image_path", write_image_path, "default");  
//============== Read ros parameter =====================//

    uav_config = make_shared<ConfigParser>(uav_config_file);

//===== Read camera intrinsics and extrinsic with yaml-cpp =====//
    T_camera_to_image = uav_config->T_cam_image;
    if(cam == "camA"){
        cameraMatrix = (cv::Mat_<double>(3, 3) << uav_config->cameraA.ir_camera.cam1.fx, 0, uav_config->cameraA.ir_camera.cam1.cx, 0, uav_config->cameraA.ir_camera.cam1.fy, uav_config->cameraA.ir_camera.cam1.cy, 0, 0, 1);
    for (int i = 0; i < landmark_num; ++i) {
        distCoeffs.emplace_back(uav_config->cameraA.ir_camera.cam1.D[i]);
    }
    }
    if(cam == "camB"){
        cameraMatrix = (cv::Mat_<double>(3, 3) << uav_config->cameraB.ir_camera.cam1.fx, 0, uav_config->cameraB.ir_camera.cam1.cx, 0, uav_config->cameraB.ir_camera.cam1.fy, uav_config->cameraB.ir_camera.cam1.cy, 0, 0, 1);
    for (int i = 0; i < landmark_num; ++i) {
        distCoeffs.emplace_back(uav_config->cameraB.ir_camera.cam1.D[i]);
    }
    }
    if(cam == "camC"){
        cameraMatrix = (cv::Mat_<double>(3, 3) << uav_config->cameraC.ir_camera.cam1.fx, 0, uav_config->cameraC.ir_camera.cam1.cx, 0, uav_config->cameraC.ir_camera.cam1.fy, uav_config->cameraC.ir_camera.cam1.cy, 0, 0, 1);
    for (int i = 0; i < landmark_num; ++i) {
        distCoeffs.emplace_back(uav_config->cameraC.ir_camera.cam1.D[i]);
    }
    }
    if(cam == "camD"){
        cameraMatrix = (cv::Mat_<double>(3, 3) << uav_config->cameraD.ir_camera.cam1.fx, 0, uav_config->cameraD.ir_camera.cam1.cx, 0, uav_config->cameraD.ir_camera.cam1.fy, uav_config->cameraD.ir_camera.cam1.cy, 0, 0, 1);
    for (int i = 0; i < landmark_num; ++i) {
        distCoeffs.emplace_back(uav_config->cameraD.ir_camera.cam1.D[i]);
    }
    }
//===== Read camera intrinsics and extrinsic with yaml-cpp =====//


//======================== Read IRlandmark extrinsic ========================//
    T_IRLandmark_to_drone = uav_config->ir_landmark.T_drone_IRLandmark.inverse();
    landmark_num = uav_config->ir_landmark.number;
    for (int i = 0; i < landmark_num; ++i) {
        drone_landmarks_cv.emplace_back(cv::Point3f(uav_config->ir_landmark.layout(0,i)/1000.0, uav_config->ir_landmark.layout(1,i)/1000.0, uav_config->ir_landmark.layout(2,i)/1000.0));
    }
//======================== Read IRlandmark extrinsic ========================//

    
//============================= Initialize ROS topic =============================//
    sub_marker_pixel = nh.subscribe("/" + cam + "/single_cam_process_ros/ir_mono/marker_pixel", 1, &PnPTargetNodeROS::ir_marker_pixel_cb, this);
    sub_drone_vio_pose = nh.subscribe("/vio", 1, &PnPTargetNodeROS::drone_vio_pose_cb, this);
    sub_drone_vicon_pose = nh.subscribe("/mocap", 1, &PnPTargetNodeROS::drone_vicon_pose_cb, this);
    sub_drone_imu = nh.subscribe("/imu", 1, &PnPTargetNodeROS::drone_imu_cb, this);
    
    pub_cam_to_estimation = nh.advertise<geometry_msgs::TransformStamped>("/" + cam + "/single_cam_process_ros/ir_mono/T_cam_to_estimation", 1);
    pub_drone_vicon_pose = nh.advertise<geometry_msgs::PoseStamped>("vicon/pose_correct", 1);
    // pub_target_pose_from_img = nh.advertise<geometry_msgs::PoseStamped>("pnp_trt/topic_target_pose_from_img", 1);
    // pub_target_pose_from_img_filter = nh.advertise<geometry_msgs::PoseStamped>("pnp_trt/topic_target_pose_from_img_filter", 1);
    // pub_target_pose_in_base = nh.advertise<geometry_msgs::PoseStamped>("mulcam_pnp/topic_target_pose_in_body", 1);
    // pub_relative_pose_mocap =  nh.advertise<geometry_msgs::PoseStamped>("mulcam_pnp/relative_pose_cam2target_mocap", 1);

    // pub_drone_model = nh.advertise<visualization_msgs::MarkerArray>("/drone_model", 1);
}   


//============================= Initialize ROS topic =============================//


void PnPTargetNodeROS::drone_vio_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    if(!drone_pose_vicon_first_flag){
        //在有vicon初值情况下，需要将VIO的起点加上vicon初值，作为飞机位置。
        drone_pose.pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        drone_pose.pos += drone_pose_vicon_init.pos;
        //姿态在VIO和Vicon坐标系下都是0
        drone_pose.Quat = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    }else{
        //在没有vicon值情况下，VIO起点就是自身摆放的起点。
        drone_pose.pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        drone_pose.Quat = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    }
  //    printf("drone_attitude.Quat = %f, %f, %f, %f\n", drone_attitude.Quat.w(), drone_attitude.Quat.x(), drone_attitude.Quat.y(), drone_attitude.Quat.z());
}

void PnPTargetNodeROS::drone_vicon_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    drone_pose_vicon.pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    drone_pose_vicon.Quat = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    drone_pose_vicon.Quat = Eigen::Quaterniond(drone_pose_vicon.Quat.toRotationMatrix() * uav_config->Vicon_correction);
}

void PnPTargetNodeROS::base_vicon_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    base_pose_vicon.pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    base_pose_vicon.Quat = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    base_pose_vicon.Quat = Eigen::Quaterniond(base_pose_vicon.Quat.toRotationMatrix() * uav_config->Vicon_correction);
}

void PnPTargetNodeROS::drone_imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
    drone_attitude = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    // if(drone_attitude_init_flag){
    //   drone_attitude_init = drone_attitude;
    //   drone_attitude_init_flag = false;
    //   //只需要把四元数的yaw方向的置为0即可，roll和pitch不需要置为0
    //   Eigen::Vector3d euler_init = drone_attitude_init.toRotationMatrix().eulerAngles(2,1,0);
    //   //只把yaw对应四元数取出来，作为yaw角偏置，然后乘到当前四元数上，就可以把当前四元数的yaw置为0
    //   drone_attitude_init = Eigen::Quaterniond(cos(euler_init[0]/2), 0, 0, sin(euler_init[0]/2));
    // }
    // drone_attitude = drone_attitude_init.inverse() * drone_attitude;//只矫正yaw角

//     ImuData message;
//     message.timestamp = msg->header.stamp.toSec();
//     message.q = drone_attitude;
//     message.am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

//     filter->agent_self.feed_measurement_imu(message);
}

//============================= marker_pixel receive =============================//
void PnPTargetNodeROS::ir_marker_pixel_cb(const multi_camera_cooperation::landmark::ConstPtr &msg){
    stamp = msg->header.stamp;
    for (int i = 0; i < msg->x.size(); i++)
        marker_pixels.emplace_back(cv::Point2f(msg->x[i], msg->y[i]));
    landmark_pose_solve();
}
//============================= marker_pixel receive =============================//


void PnPTargetNodeROS::landmark_pose_solve(){
    //进行PnP解算
    pnpGoodFlag = pnp_process(marker_pixels);
    if(!pnpGoodFlag){
        marker_pixels.clear();
    }


//    pub_target_pose_from_img.publish(msg_target_pose_from_img);
//  ros::Time t = ros::Time().fromSec(stamp.toSec() - PnP_time_delay);//PnP检测中，由相机捕获、程序处理、滤波等带来的时间延迟
/*
  msg_target_pose_from_img_filter.header.stamp = stamp; //直接赋予ir_img的时间戳
  msg_target_pose_from_img_filter.pose.position.x = target_pos_in_img[0];
  msg_target_pose_from_img_filter.pose.position.y = target_pos_in_img[1];
  msg_target_pose_from_img_filter.pose.position.z = target_pos_in_img[2];
  msg_target_pose_from_img_filter.pose.orientation.w = target_q_in_img.w();
  msg_target_pose_from_img_filter.pose.orientation.x = target_q_in_img.x();
  msg_target_pose_from_img_filter.pose.orientation.y = target_q_in_img.y();
  msg_target_pose_from_img_filter.pose.orientation.z = target_q_in_img.z();
  pub_target_pose_from_img_filter.publish(msg_target_pose_from_img_filter);
*/
//marker在camera下的位姿
#ifdef USE_IMU_DIFF
    T_image_to_markers.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(); // landmark在相机下的姿态暂定为Identity()
#else
    T_image_to_markers.block<3, 3>(0, 0) = target_q_in_img.toRotationMatrix(); // 改成landmark在相机下的姿态
#endif
    T_image_to_markers.block<3, 1>(0, 3) = target_pos_in_img;

    // // 对于T_image_to_markers进行离群值滤除
    // if(pnpGoodFlag) {
    //     if(window_T_image_to_markers.empty()) {
    //         window_T_image_to_markers.push_back(T_image_to_markers);
    //     }
    //     else {
    //         if(window_T_image_to_markers.size()==5) {
    //             window_T_image_to_markers.pop_front();
    //         }
    //         Eigen::Quaterniond avg_quat(0,0,0,0);
    //         Eigen::Vector3d avg_trans(0,0,0);
    //         for(deque<Eigen::Matrix4d>::iterator it = window_T_image_to_markers.begin(); it!=window_T_image_to_markers.end(); it++) {
    //             Eigen::Matrix3d rotation_matrix = (*it).block<3, 3>(0, 0);
    //             Eigen::Quaterniond quat(rotation_matrix);
    //             Eigen::Vector3d translation = (*it).block<3, 1>(0, 3);
    //             avg_quat = avg_quat.slerp(1.0 / window_T_image_to_markers.size(), quat);
    //             avg_trans += translation / window_T_image_to_markers.size();
    //         }
    //         if((T_image_to_markers.block<3, 1>(0, 3)-avg_trans).norm() >= 0.001) {
    //             Eigen::Matrix3d avg_rot(avg_quat);
    //             T_image_to_markers.block<3, 3>(0, 0) = avg_rot;
    //             T_image_to_markers.block<3, 1>(0, 3) = avg_trans;
    //             // T_image_to_markers(0,3) = 0;
    //             // T_image_to_markers(1,3) = 0;
    //             // T_image_to_markers(2,3) = 0;
    //         }
    //         window_T_image_to_markers.push_back(T_image_to_markers);
    //     }
    // }
    // else {
    //     while(!window_T_image_to_markers.empty()) {
    //         window_T_image_to_markers.pop_front();
    //     }
    // }

    //先从相机坐标系到图像坐标系，然后从图像坐标系到标记点坐标系，再由标记点坐标系到飞机坐标系
    T_cam_to_estimation = T_camera_to_image * T_image_to_markers * T_IRLandmark_to_drone;
    // T_body_to_drone.block<3,1>(0,3) = T_body_to_drone.block<3,1>(0,3) + T_markers_to_drone.block<3,1>(0,3); //marker在drone1坐标系下的位置
    R_cam_to_estimation = T_cam_to_estimation.block<3, 3>(0, 0);

    if((T_cam_to_estimation.block<3, 1>(0, 3)).norm() > 3.0) {
        pnpGoodFlag = false;
        marker_pixels.clear();
    }

    t_cam_to_estimation = EigenVector3dToTFVector3(T_cam_to_estimation.block<3, 1>(0, 3));
    q_cam_to_estimation = EigenQuaterniondToTFQuaternion(Eigen::Quaterniond(R_cam_to_estimation));
    

//=====================================写入文件进一步分析======================================//
    t_cam_to_estimation_file.open("/home/hezijia/catkin_ws/src/multi_camera_cooperation/data/t_body_to_drone_camC.txt",ios::out|ios::app);
	//输入你想写入的内容 
	t_cam_to_estimation_file<<t_cam_to_estimation[0]<<" "<<t_cam_to_estimation[1]<<" "<<t_cam_to_estimation[2]<<endl;
	t_cam_to_estimation_file.close();

    q_cam_to_estimation_file.open("/home/hezijia/catkin_ws/src/multi_camera_cooperation/data/q_cam_to_estimation_camC.txt",ios::out|ios::app);
	//输入你想写入的内容 
	q_cam_to_estimation_file<<q_cam_to_estimation.w()<<" "<<q_cam_to_estimation.x()<<" "<<q_cam_to_estimation.y()<<" "<<q_cam_to_estimation.z()<<endl;
	q_cam_to_estimation_file.close();
//=====================================写入文件进一步分析======================================//

    printf(GREEN "[PNP] t_cam_to_estimation = %.3f, %.3f, %.3f | q_cam_to_estimation (wxyz) = %.3f, %.3f, %.3f, %.3f\n" RESET,
         t_cam_to_estimation.x(), t_cam_to_estimation.y(), t_cam_to_estimation.z(),
         q_cam_to_estimation.getW(), q_cam_to_estimation.getX(), q_cam_to_estimation.getY(), q_cam_to_estimation.getZ());

    // 发布单台相机PNP估计结果T_cam_to_estimation
    if(pnpGoodFlag){
        cam_to_estimation.setOrigin(t_cam_to_estimation);
        
		cam_to_estimation.setRotation(q_cam_to_estimation);
        cam_to_estimation.stamp_ = ros::Time::now();
        cam_to_estimation.frame_id_ = cam;
        cam_to_estimation.child_frame_id_ = "Estimationfrom"+cam;
        tf::transformStampedTFToMsg(cam_to_estimation, msg_T_cam_to_estimation);
        pub_cam_to_estimation.publish(msg_T_cam_to_estimation);
        br0->sendTransform(msg_T_cam_to_estimation);
    }


// //======================================= Ground Truth ============================================================//
//     Eigen::Vector3d t_body_to_drone_gt = body_pose_vicon.Quat.inverse() * (drone_pose_vicon.pos - body_pose_vicon.pos);//转换到body坐标系
// //  Eigen::Vector3d t_body_to_drone_gt = drone_neighbour_pose_vicon.pos - drone_pose_vicon.pos;//Vicon坐标系
//     Eigen::Quaterniond q_body_to_drone_gt = body_pose_vicon.Quat.inverse() * drone_pose_vicon.Quat;
// //  Eigen::Vector3d euler_gt = quaternion2euler(q_body_to_drone_gt.x(), q_body_to_drone_gt.y(), q_body_to_drone_gt.z(), q_body_to_drone_gt.w());
//   printf(GREEN "[GT] t_body_to_drone = %.3f, %.3f, %.3f | q_body_to_drone (wxyz) = %.3f, %.3f, %.3f, %.3f\n" RESET,
//          t_body_to_drone_gt[0], t_body_to_drone_gt[1], t_body_to_drone_gt[2],
//          q_body_to_drone_gt.w(), q_body_to_drone_gt.x(), q_body_to_drone_gt.y(), q_body_to_drone_gt.z());
//   geometry_msgs::PoseStamped msg_relative_pose;
//   msg_relative_pose.header.stamp = stamp;
//   msg_relative_pose.pose.position.x = t_body_to_drone_gt[0];
//   msg_relative_pose.pose.position.y = t_body_to_drone_gt[1];
//   msg_relative_pose.pose.position.z = t_body_to_drone_gt[2];
//   msg_relative_pose.pose.orientation.w = q_body_to_drone_gt.w();
//   msg_relative_pose.pose.orientation.x = q_body_to_drone_gt.x();
//   msg_relative_pose.pose.orientation.y = q_body_to_drone_gt.y();
//   msg_relative_pose.pose.orientation.z = q_body_to_drone_gt.z();
//   pub_relative_pose_mocap.publish(msg_relative_pose);
// //======================================= Ground Truth ============================================================//

}


bool PnPTargetNodeROS::T_shape_identify(vector<cv::Point2f> &pointsVector){
    marker_pixels_sorted.clear();
    marker_pixels_up.clear();
    marker_pixels_down.clear();
    float slope1[3] = {0};
    bool TShapeGoodFlag = true;   

    for(int i = 0; i < 4; i++){
        Eigen::Vector2d linkvector0 = subtractPoints(pointsVector[i], pointsVector[(i + 1) % 4]);
        Eigen::Vector2d linkvector1 = subtractPoints(pointsVector[i], pointsVector[(i + 2) % 4]);
        Eigen::Vector2d linkvector2 = subtractPoints(pointsVector[i], pointsVector[(i + 3) % 4]);
        if((abs(vectorAngle(linkvector0, linkvector1, 1)) < 10) || (abs(vectorAngle(linkvector0, linkvector1, 1) - 180) < 10)){
            marker_pixels_up.emplace_back(pointsVector[i]);
        }else if((abs(vectorAngle(linkvector1, linkvector2, 1)) < 10) || (abs(vectorAngle(linkvector1, linkvector2, 1) - 180) < 10)){
            marker_pixels_up.emplace_back(pointsVector[i]);
        }else if((abs(vectorAngle(linkvector2, linkvector0, 1)) < 10) || (abs(vectorAngle(linkvector2, linkvector0, 1) - 180) < 10)){
            marker_pixels_up.emplace_back(pointsVector[i]);
        }else{
            marker_pixels_down.emplace_back(pointsVector[i]);
        }
    }

    if((marker_pixels_up.size() != 3) || (marker_pixels_down.size() != 1)){
        return false;
    }

    // cout<< "marker_pixels_up.size():"<<marker_pixels_up.size()<<endl;
    // cout<< "marker_pixels_down.size():"<<marker_pixels_down.size()<<endl;

    for(int i = 0; i < 3; i++){//need upgrade!!!
        Eigen::Vector2d linkvector0 = subtractPoints(marker_pixels_up[i], marker_pixels_up[(i + 1) % 3]);
        Eigen::Vector2d linkvector1 = subtractPoints(marker_pixels_up[i], marker_pixels_up[(i + 2) % 3]);
        
        cout<<vectorAngle(linkvector0, linkvector1, 1) - 180<<endl;
        if(abs(vectorAngle(linkvector0, linkvector1, 1) - 180) < 30){
            marker_pixels_sorted.emplace_back(marker_pixels_up[i]);
            cout << marker_pixels_sorted << endl;
            Eigen::Vector2d linkvector =  subtractPoints(marker_pixels_up[i], marker_pixels_down[0]);
            if(checkRotationDirection(linkvector, linkvector0)){
                marker_pixels_sorted.emplace_back(marker_pixels_up[(i + 1) % 3]);
                marker_pixels_sorted.emplace_back(marker_pixels_up[(i + 2) % 3]);
                marker_pixels_sorted.emplace_back(marker_pixels_down[0]);
            }else{
                marker_pixels_sorted.emplace_back(marker_pixels_up[(i + 2) % 3]);
                marker_pixels_sorted.emplace_back(marker_pixels_up[(i + 1) % 3]);
                marker_pixels_sorted.emplace_back(marker_pixels_down[0]);
            }
        }
    }

    pointsVector.clear();
    pointsVector = marker_pixels_sorted;  

    if(pointsVector.size() != landmark_num)  
        TShapeGoodFlag = false;

    Eigen::Vector2d vec10 = subtractPoints(pointsVector[1], pointsVector[0]);
    Eigen::Vector2d vec20 = subtractPoints(pointsVector[2], pointsVector[0]);
    if(abs(vectorNorm2D(vec10) - vectorNorm2D(vec20)) > 30)
        TShapeGoodFlag = false;

    if(TShapeGoodFlag){
        marker_pixels_buffer.clear();
        marker_pixels_buffer =  pointsVector;
        return true;
    }else{
        cout<< "marker_pixels_sort.size():"<<marker_pixels_sorted.size()<<endl;
        pointsVector = marker_pixels_buffer;
        return false;
    }

    
}

bool PnPTargetNodeROS::pnp_process(vector<cv::Point2f> &pointsVector){
    ROS_INFO("pnp_process start!");

#ifdef USE_4_Point
    if(pointsVector.size() == landmark_num){
        if(!T_shape_identify(pointsVector)){
            return false;
        }
    }else{
        return false;
    }
#endif

    //solvePnP
    //solvePnP(drone_landmarks_cv, pointsVector, cameraMatrix, distCoeffs, outputRvecRaw, outputTvecRaw, false, cv::SOLVEPNP_IPPE);
    solvePnP(drone_landmarks_cv, pointsVector, cameraMatrix, distCoeffs, outputRvecRaw, outputTvecRaw, false);
    Eigen::Vector3d eulerAngles;
    getEulerAngles(outputRvecRaw, eulerAngles, target_q_in_img);
    target_pos_in_img << outputTvecRaw.val[0], outputTvecRaw.val[1], outputTvecRaw.val[2];//转到相机坐标系下
    printf(YELLOW "[PnP Solve target] x: %.3f, y: %.3f, z: %.3f\n" RESET, target_pos_in_img[0], target_pos_in_img[1], target_pos_in_img[2]);
    msg_target_pose_from_img.header.stamp = ros::Time::now();
    target_pos_in_img[0] = min(max(target_pos_in_img[0], -20.0), 20.0);
    target_pos_in_img[1] = min(max(target_pos_in_img[1], -10.0), 10.0);
    target_pos_in_img[2] = min(max(target_pos_in_img[2], -10.0), 10.0);
    //范围约束
    //均值滤波器
    //低通滤波器
//    msg_target_pose_from_img_filter_lp.header.stamp = stamp;
//    double filter_z_lp = FilterPosZLp.low_pass_filter(filter_z_mean);
//    msg_target_pose_from_img_filter_lp.pose.position.z = filter_z_lp;
//    std::cout << "************ filter_z_mean = " << filter_z_mean << "\tfilter_z_lp = " << filter_z_lp << std::endl;

//    msg_target_pose_from_img_filter.header.stamp = stamp;
//    msg_target_pose_from_img_filter.pose.position.x = FilterPosX.sgfilter(FilterPosX.limit_jump_filter(msg_target_pose_from_img.pose.position.x,0.02));
//    msg_target_pose_from_img_filter.pose.position.y = FilterPosY.sgfilter(FilterPosY.limit_jump_filter(msg_target_pose_from_img.pose.position.y, 0.02));
//    msg_target_pose_from_img_filter.pose.position.z = min(max(FilterPosZ.limit_jump_filter(msg_target_pose_from_img.pose.position.z,0.02), 0.5), 4.0);

    //20231013wzy disable this because the optical flow detection is not continuous.
//    double x_avg = FilterPosX.filter_avg(FilterPosX.limit_jump_filter(target_pos_in_img[0],0.02));
//    double y_avg = FilterPosY.filter_avg(FilterPosY.limit_jump_filter(target_pos_in_img[1], 0.02));
//    double z_avg = FilterPosZ.filter_avg(FilterPosZ.limit_jump_filter(target_pos_in_img[2], 0.02));
//    printf(YELLOW "[PnP filter_avg] x: %.3f, y: %.3f, z: %.3f \n" RESET, x_avg, y_avg, z_avg);
//    target_pos_in_img[0] = min(max(x_avg, -2.0), 2.0);
//    target_pos_in_img[1] = min(max(y_avg, -2.0), 2.0);
//    target_pos_in_img[2] = min(max(z_avg, 0.5), 4.0);
//    printf(YELLOW "[PnP min max] x: %.3f, y: %.3f, z: %.3f \n" RESET, target_pos_in_img[0], target_pos_in_img[1], target_pos_in_img[2]);
    return true;
}


