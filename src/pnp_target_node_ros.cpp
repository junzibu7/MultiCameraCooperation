//
// Created by hzj on 24-2-28.
//
#include "multi_camera_cooperation/pnp_target_node_ros.h"

using namespace std;

PnPTargetNodeROS::PnPTargetNodeROS(){
    
}

void PnPTargetNodeROS::init(ros::NodeHandle &nh){

//============== Read ros parameter =====================//
    std::string uav_config_file;
    int drone_id;

    nh.param<double>("PnP_time_delay", PnP_time_delay, 0.0);//PnP检测中，由相机捕获、程序处理、滤波等带来的时间延迟
    nh.param<int>("ir_binary_threshold", ir_binary_threshold, 50);
    nh.param<int>("drone_id", drone_id, 0);
    nh.param<std::string>("uav_config_file", uav_config_file, "default");
    nh.param<std::string>("write_image_path", write_image_path, "default");  
//============== Read ros parameter =====================//

    uav_config = make_shared<ConfigParser>(uav_config_file);

//===== Read camera intrinsics and extrinsic with yaml-cpp =====//
    T_body_to_camera = uav_config->cameraA.color_camera.T_body_camColor * uav_config->cameraA.ir_camera.cam1.T_camColor_camIR1 * uav_config->T_cam_image;
    cameraMatrix = (cv::Mat_<double>(3, 3) << uav_config->cameraA.ir_camera.cam1.fx, 0, uav_config->cameraA.ir_camera.cam1.cx, 0, uav_config->cameraA.ir_camera.cam1.fy, uav_config->cameraA.ir_camera.cam1.cy, 0, 0, 1);
    for (int i = 0; i < 5; ++i) {
        distCoeffs.emplace_back(uav_config->cameraA.ir_camera.cam1.D[i]);
    }
//===== Read camera intrinsics and extrinsic with yaml-cpp =====//


//======================== Read IRlandmark extrinsic ========================//
    T_markers_to_drone = uav_config->ir_landmark.T_marker_IRLandmark.inverse();
    landmark_num = uav_config->ir_landmark.number;
    for (int i = 0; i < landmark_num; ++i) {
        drone_landmarks_cv.emplace_back(cv::Point3f(uav_config->ir_landmark.layout(0,i)/1000.0, uav_config->ir_landmark.layout(1,i)/1000.0, uav_config->ir_landmark.layout(2,i)/1000.0));
    }
//======================== Read IRlandmark extrinsic ========================//

    
//============================= Initialize ROS topic =============================//
#ifdef IMG_COMPRESSED
    sub_ir_img = nh.subscribe("/ir_mono", 1, &PnPTargetNodeROS::ir_compressed_img_cb, this);
#else
    sub_ir_img = nh.subscribe("/ir_mono", 1, &PnPTargetNodeROS::ir_raw_img_cb, this);
#endif
    sub_drone_vio_pose = nh.subscribe("/vio", 1, &PnPTargetNodeROS::drone_vio_pose_cb, this);
    sub_drone_vicon_pose = nh.subscribe("/mocap", 1, &PnPTargetNodeROS::drone_vicon_pose_cb, this);
    sub_drone_imu = nh.subscribe("/imu", 1, &PnPTargetNodeROS::drone_imu_cb, this);
    
#ifdef SHOW_ORIGIN_IMG
    pub_ir_img = nh.advertise<sensor_msgs::Image>("ir_mono/origin",1);
#endif
    pub_marker_pixel = nh.advertise<multi_camera_cooperation::Markers>("/marker_pixel",10);
    // pub_relative_attitude_by_marker_pixel = nh.advertise<geometry_msgs::PoseStamped>("relative_pose_by_marker_pixel",10);
    pub_drone_vicon_pose = nh.advertise<geometry_msgs::PoseStamped>("vicon/pose_correct", 1);
    pub_ir_show_img = nh.advertise<sensor_msgs::Image>("ir_mono/show",1);
    // pub_ir_crop_img = nh.advertise<sensor_msgs::Image>("ir_mono/ir_crop",1);
    // pub_ir_binary_img = nh.advertise<sensor_msgs::Image>("ir_mono/ir_binary",1);
    // pub_ir_erode_dilate_img = nh.advertise<sensor_msgs::Image>("ir_mono/ir_erode_dilate",1);
    // pub_target_pose_from_img = nh.advertise<geometry_msgs::PoseStamped>("pnp_trt/topic_target_pose_from_img", 1);
    // pub_target_pose_from_img_filter = nh.advertise<geometry_msgs::PoseStamped>("pnp_trt/topic_target_pose_from_img_filter", 1);
    pub_target_pose_in_body = nh.advertise<geometry_msgs::PoseStamped>("mulcam_pnp/topic_target_pose_in_body", 1);
    // pub_target_pose_in_enu = nh.advertise<geometry_msgs::PoseStamped>("pnp_trt/topic_target_pose_in_enu", 1);
    // pub_target_pose_in_enu_vicon = nh.advertise<geometry_msgs::PoseStamped>("pnp_trt/topic_target_pose_in_enu_vicon", 1);
    pub_relative_pose_mocap =  nh.advertise<geometry_msgs::PoseStamped>("mulcam_pnp/relative_pose_cam2target_mocap", 1);


    // pub_drone_model = nh.advertise<visualization_msgs::MarkerArray>("/drone_model", 1);
    
    if(uav_config->uav_name == "kun0"){
      printf(YELLOW "[kun0] Publisher landmark_pose_solve: %s\n", pub_target_pose_in_body.getTopic().c_str());
    }
    
    ROS_INFO("Publisher subscriber initialized");
    ROS_INFO("[SWARM_DETECT] Finish initialize swarm detector, wait for data...");

    cv_ptr_compressed_ir = boost::make_shared<cv_bridge::CvImage>();
    cv_ptr_raw_ir = boost::make_shared<cv_bridge::CvImage const>();

    // //是否从launch中加载yaw角初值
    // if(yaw_relative_init !=0.0){
    //   printf(BOLDWHITE "USE pre-defined yaw offset: %f\n", yaw_relative_init);
    //   yaw_initialized_flag = true;
    //   marker_pixel_yaw_relative = yaw_relative_init;
    // }else{
    //   yaw_initialized_flag = false;
    // }
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

void PnPTargetNodeROS::body_vicon_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    body_pose_vicon.pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    body_pose_vicon.Quat = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    body_pose_vicon.Quat = Eigen::Quaterniond(body_pose_vicon.Quat.toRotationMatrix() * uav_config->Vicon_correction);
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

void PnPTargetNodeROS::ir_compressed_img_cb(const sensor_msgs::CompressedImage::ConstPtr &msg){
//    ROS_INFO("ir_img_cb");
//    std::cout << "ir timestamp = " << msg->header.stamp << std::endl;
    stamp = msg->header.stamp;
    cv_ptr_compressed_ir = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);
    ir_img = cv_ptr_compressed_ir->image;
//    cv::equalizeHist(ir_img, ir_img); //对图像均衡化
//    ir_img = cv::imdecode(msg->data, cv::IMREAD_GRAYSCALE);
#ifdef SHOW_ORIGIN_IMG
    auto ir_img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", ir_img).toImageMsg();
    pub_ir_img.publish(ir_img_msg);
#endif
    // if(!yaw_initialized_flag){
    //   yaw_initialized_flag = init_relative_yaw();
    //   return;
    // }
    if(ir_img.empty()){
        printf(RED"Not receive valid ir_img\n");
        return;
    }
    landmark_pose_solve();
}

void PnPTargetNodeROS::ir_raw_img_cb(const sensor_msgs::Image::ConstPtr &msg){
//  ROS_INFO("ir_img_cb");
//  std::cout << "ir timestamp = " << msg->header.stamp << std::endl;
    stamp = msg->header.stamp;
    cv_bridge::CvImageConstPtr cv_ptr_raw_ir;
    cv_ptr_raw_ir = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::MONO8);
    ir_img = cv_ptr_raw_ir->image;
//  cv::equalizeHist(ir_img, ir_img); //对图像均衡化
#ifdef SHOW_ORIGIN_IMG
    auto ir_img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", ir_img).toImageMsg();
    pub_ir_img.publish(ir_img_msg);
#endif
    if(ir_img.empty()){
        printf(RED"Not receive valid ir_img\n");
        return;
    }
    landmark_pose_solve();
}

void PnPTargetNodeROS::landmark_pose_solve(){
    ir_img_color_show = cv::Mat::zeros(ir_img.size(), CV_8UC3);
    cv::cvtColor(ir_img, ir_img_color_show, CV_GRAY2BGR);

    //光流追踪
    if(opticalReadyFlag){
        opticalGoodFlag = optical_flow(ir_img,marker_pixels);
    }
    //如果光流追踪失败，则重新清除marker,然后进行ROI
    if(!opticalGoodFlag){
        ROS_WARN("optical flow fails, try to roi detect!");
        marker_pixels.clear();
        roiGoodFlag = ir_img_process(ir_img, marker_pixels, 1);
    }
    //如果光流或者ROI得到 marker_pixels,进行PnP解算
    if(opticalGoodFlag || roiGoodFlag){
        pnpGoodFlag = pnp_process(marker_pixels);
        if(pnpGoodFlag){
            opticalReadyFlag = true;
        }else{
            opticalReadyFlag = false;
            marker_pixels.clear();
        }
    }else{
        pnpGoodFlag = false;
        printf(YELLOW "roi process failed.\n");
          return;
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
    T_camera_to_markers.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(); // landmark在相机下的姿态暂定为Identity()
#else
    T_camera_to_markers.block<3, 3>(0, 0) = target_q_in_img.toRotationMatrix(); // 改成landmark在相机下的姿态
#endif
    T_camera_to_markers.block<3, 1>(0, 3) = target_pos_in_img;


    //得到drone在body坐标系的位置
    T_body_to_drone = T_body_to_camera * T_camera_to_markers * T_markers_to_drone;
    R_body_to_drone = T_body_to_drone.block<3, 3>(0, 0);
#ifdef USE_IMU_DIFF
    q_body_to_drone = yaw_init_offset * q_body_to_drone;//将初始用视觉marker pixel得到的yaw角度加进去
#else
    q_body_to_drone = Eigen::Quaterniond(R_body_to_drone);
#endif
    t_body_to_drone = T_body_to_drone.block<3, 1>(0, 3);
    printf(YELLOW "[PnP] target_pos_in_img = %.3f, %.3f, %.3f\n" RESET,
        target_pos_in_img[0], target_pos_in_img[1], target_pos_in_img[2]);

//======================================= Ground Truth ============================================================//
    Eigen::Vector3d t_body_to_drone_gt = body_pose_vicon.Quat.inverse() * (drone_pose_vicon.pos - body_pose_vicon.pos);//转换到body坐标系
//  Eigen::Vector3d t_body_to_drone_gt = drone_neighbour_pose_vicon.pos - drone_pose_vicon.pos;//Vicon坐标系
    Eigen::Quaterniond q_body_to_drone_gt = body_pose_vicon.Quat.inverse() * drone_pose_vicon.Quat;
//  Eigen::Vector3d euler_gt = quaternion2euler(q_body_to_drone_gt.x(), q_body_to_drone_gt.y(), q_body_to_drone_gt.z(), q_body_to_drone_gt.w());
  printf(GREEN "[GT] t_body_to_drone = %.3f, %.3f, %.3f | q_body_to_drone (wxyz) = %.3f, %.3f, %.3f, %.3f\n" RESET,
         t_body_to_drone_gt[0], t_body_to_drone_gt[1], t_body_to_drone_gt[2],
         q_body_to_drone_gt.w(), q_body_to_drone_gt.x(), q_body_to_drone_gt.y(), q_body_to_drone_gt.z());
  geometry_msgs::PoseStamped msg_relative_pose;
  msg_relative_pose.header.stamp = stamp;
  msg_relative_pose.pose.position.x = t_body_to_drone_gt[0];
  msg_relative_pose.pose.position.y = t_body_to_drone_gt[1];
  msg_relative_pose.pose.position.z = t_body_to_drone_gt[2];
  msg_relative_pose.pose.orientation.w = q_body_to_drone_gt.w();
  msg_relative_pose.pose.orientation.x = q_body_to_drone_gt.x();
  msg_relative_pose.pose.orientation.y = q_body_to_drone_gt.y();
  msg_relative_pose.pose.orientation.z = q_body_to_drone_gt.z();
  pub_relative_pose_mocap.publish(msg_relative_pose);
//======================================= Ground Truth ============================================================//


#ifdef ENABLE_VISUALIZATION
  //可视化求解情况
  cv::rectangle(ir_img_color_show, cv::Point2f(0,0), cv::Point2f(640, 20), cv::Scalar(20, 20, 20), -1);
  cv::putText(ir_img_color_show,
              "x: " + Convert(target_pos_in_img.x()) + " y: " + Convert(target_pos_in_img.y())  + " z: " + Convert(target_pos_in_img.z()),
              cv::Point(0, 20), cv::FONT_HERSHEY_TRIPLEX ,0.65,cv::Scalar(255,255,255),1,false);

  if(opticalGoodFlag){
    cv::putText(ir_img_color_show,
                "track",
                cv::Point(340, 20), cv::FONT_HERSHEY_TRIPLEX ,0.65,cv::Scalar(0,255,0),1,false);
  }else{
    cv::putText(ir_img_color_show,
                "track",
                cv::Point(340, 20), cv::FONT_HERSHEY_TRIPLEX ,0.65,cv::Scalar(0,0,255),1,false);
  }

  if(roiGoodFlag) {
    cv::putText(ir_img_color_show,
                "roi",
                cv::Point(420, 20), cv::FONT_HERSHEY_TRIPLEX, 0.65, cv::Scalar(0, 255, 0), 1,  false);
  }else{
    cv::putText(ir_img_color_show,
                "roi",
                cv::Point(420, 20), cv::FONT_HERSHEY_TRIPLEX, 0.65, cv::Scalar(0, 0, 255), 1, false);
  }

  if(pnpGoodFlag){
    cv::putText(ir_img_color_show,
                "pnp",
                cv::Point(470, 20), cv::FONT_HERSHEY_TRIPLEX ,0.65,cv::Scalar(0,255,0),1,false);
  }else{
    cv::putText(ir_img_color_show,
                "pnp",
                cv::Point(470, 20), cv::FONT_HERSHEY_TRIPLEX ,0.65,cv::Scalar(0,0,255),1,false);
  }

  if(yoloGoodFlag){
    cv::putText(ir_img_color_show,
                "yolo",
                cv::Point(530, 20), cv::FONT_HERSHEY_TRIPLEX ,0.65,cv::Scalar(0,255,0),1,false);
  }else{
    cv::putText(ir_img_color_show,
                "yolo",
                cv::Point(530, 20), cv::FONT_HERSHEY_TRIPLEX ,0.65,cv::Scalar(0,0,255),1,false);
  }

  if(!pnpGoodFlag && !yoloGoodFlag)
  {
    cv::putText(ir_img_color_show,
                "fail",
                cv::Point(580, 20), cv::FONT_HERSHEY_TRIPLEX ,0.65,cv::Scalar(0,0,250),1,false);
  }

  //画横线框，和图像中心点，校验求解目标原点是否正确
//  cv::circle(ir_img_color_show,cv::Point2f(320,240),2,cv::Scalar(255,0,0),1);
//  cv::line(ir_img_color_show,cv::Point2f(0,240),cv::Point2f(640,240),cv::Scalar(255,0,0),1);
//  cv::line(ir_img_color_show,cv::Point2f(320,0),cv::Point2f(320,480),cv::Scalar(255,0,0),1);
//  cv::namedWindow("ir_img_color_show", cv::WINDOW_NORMAL);
//  cv::resizeWindow("ir_img_color_show", 2560, 1920);
//  cv::imshow("ir_img_color_show", ir_img_color_show);
//  cv::waitKey(1);
  auto ir_show_msg_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", ir_img_color_show).toImageMsg();
  pub_ir_show_img.publish(ir_show_msg_img);
#endif
}


bool PnPTargetNodeROS::ir_img_process(cv::Mat &_ir_img, vector<cv::Point2f> &pointsVector, int model){
    ROS_WARN("ir_img_process function");
    if(_ir_img.empty())
    {
        ROS_ERROR("ir_img_process, NO _ir_img !!!");
        return false;
    }

    if(model == 1){//尝试用FAST检测特征点
        if(extractFeatures(ir_img, pointsVector)){
            printf("[roi process] by extractFeatures get %zu contours\n", pointsVector.size());
            return true;
        }else{
            ROS_ERROR("[roi process] the number of contours is %zu but should be %d", ir_contours_final.size(), landmark_num);
            return false;
        }
    }

// =====================need complete ====//
    if(model == 2){//尝试用binary检测特征点
        if(extractFeatures(ir_img, pointsVector)){
            printf("[roi process] by extractFeatures get %zu contours\n", pointsVector.size());
            return true;
        }else{
            ROS_ERROR("[roi process] the number of contours is %zu but should be %d", ir_contours_final.size(), landmark_num);
            return false;
        }
    }
// =====================need complete ====//

        /**之前的寻找轮廓的方法**/
        /*{
        int try_count = 0;
        while(1){
            //二值化及膨胀腐蚀
            threshold(ir_img_crop, ir_binary, ir_binary_threshold, 255, cv::THRESH_BINARY);
            cv::erode(ir_binary, ir_erode, erodeElement);
//            auto ir_erode_dilate_img = cv_bridge::CvImage(std_msgs::Header(), "mono8", ir_erode).toImageMsg();
//            pub_ir_erode_dilate_img.publish(ir_erode_dilate_img);

            //寻找轮廓
            findContours(ir_erode, ir_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
            cout << "contours number = " << ir_contours.size() << endl;

            if(ir_contours.size() < landmark_num){
                ir_binary_threshold -= 5;
                printf(YELLOW "[down] ir_binary_threshold set to %d\n", ir_binary_threshold);int
                if(ir_binary_threshold <= 20){
                    ir_binary_threshold = 100;
                    return false;
                }
            }else if(ir_contours.size() > landmark_num){
                ir_binary_threshold += 5;
                printf(YELLOW "[up] ir_binary_threshold set to %d\n", ir_binary_threshold);
                if(ir_binary_threshold >= 240){
                    ir_binary_threshold = 100;
                    return false;
                }
            }else{
                printf(GREEN"find %d, ir_binary_threshold set to %d\n", landmark_num, ir_binary_threshold);
                break;
            }

            if(try_count++ > 30){ //如果30次都没有找到合适阈值，就返回false
                return false;
            }
        }
    }

    //判断区域大小和横纵比是圆形灯珠区域
    ir_contours_final.clear();
    for (int i = 0; i < ir_contours.size(); ++i) {
//        cout << "area = " << cv::contourArea(ir_contours[i]) << endl;
//        if(area < 10) {
//            continue;
//        }
//        rect = boundingRect(ir_contours[i]);
//        float ratio = float(rect.width) / float(rect.height);
//        if(ratio < 1.2 && ratio >0.8){
//            ir_contours_final.push_back(ir_contours[i]);
//        }
//        cout << "**************** ratio = " << ratio << "************************" << endl;
        //20210607 change to ratio < 3 && ratio >0.6, origin is ratio < 5 && ratio >0.6
//        if(ratio < 2 && ratio >0.6){
//            ir_contours_final.push_back(ir_contours[i]);
//        }
        ir_contours_final.push_back(ir_contours[i]);
    }

    //在ROI图像上画出边框,可视化发布边框
    cv::Mat ir_binary_color_show;
    cv::cvtColor(ir_erode, ir_binary_color_show, CV_GRAY2BGR);

    for (int i = 0; i < ir_contours_final.size(); i++) {
        cv::Rect bbox;
        bbox = boundingRect(ir_contours_final[i]);
        cv::rectangle(ir_binary_color_show, bbox, cv::Scalar(0, 255, 0), 1);
    }
    auto ir_binary_msg_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", ir_binary_color_show).toImageMsg();
    pub_ir_binary_img.publish(ir_binary_msg_img);

    //对齐到原图的像素点坐标,20231022因为采用了region_mask，所以原图像左上角坐标没有改变，不对齐
//    for (int i = 0; i < ir_contours_final.size(); ++i) {
//        for (int j = 0; j < ir_contours_final[i].size(); ++j) {
//            ir_contours_final[i][j].x += roi.x;
//            ir_contours_final[i][j].y += roi.y;
//        }
//    }
//    ROS_INFO("findcontours complete");

        drawContours(ir_img_color_show, ir_contours_final, -1, (0, 255, 0), 1);

        if (ir_contours_final.size() == landmark_num) {
//        printf(GREEN "[roi process] get %zu contours\n", ir_contours_final.size());
        pointsVector.clear();
        marker_pixels_sorted.clear();
        for (int i = 0; i < ir_contours_final.size(); i++) {
            cv::Rect bbox;
            bbox = boundingRect(ir_contours_final[i]);
            pointsVector.emplace_back(cv::Point2f((bbox.tl() + bbox.br()).x / 2.0, (bbox.tl() + bbox.br()).y / 2.0));
//            std::cout << "pointsVector xy = " << pointsVector[i].x << ", " << pointsVector[i].y << std::endl;
        }
        }*/

        //依据每个点的上下左右关系确定对应的0,1,2,3,4,5号ID，按照从上到下，从左到右的顺序。
        //先上下排列，选出上下两排
//        sort(pointsVector.begin(), pointsVector.end(), marker_compare_y);
    
//         sort(pointsVector.begin(), pointsVector.end(), [=](cv::Point pt1, cv::Point pt2){return  pt1.y < pt2.y;});

// #ifdef USE_4_Point
//     std::vector<cv::Point2f> marker_pixels_up, marker_pixels_down;
//     marker_pixels_up.emplace_back(pointsVector[0]);
//     marker_pixels_up.emplace_back(pointsVector[1]);
//     marker_pixels_down.emplace_back(pointsVector[2]);
//     marker_pixels_down.emplace_back(pointsVector[3]);
// #endif
//         //从左向右排列，选出序号
// //        sort(marker_pixels_up.begin(), marker_pixels_up.end(), marker_compare_x);
// //        sort(marker_pixels_down.begin(), marker_pixels_down.end(), marker_compare_x);
//         sort(marker_pixels_up.begin(), marker_pixels_up.end(), [=](cv::Point pt1, cv::Point pt2){return  pt1.x < pt2.x;});
//         sort(marker_pixels_down.begin(), marker_pixels_down.end(), [=](cv::Point pt1, cv::Point pt2){return  pt1.x < pt2.x;});
//         //使用上面的点
//         for (int i = 0;  i < marker_pixels_up.size(); i++) {
//             marker_pixels_sorted.emplace_back(marker_pixels_up[i]);
//         }
//         //使用下面的点
//         for (int i = 0;  i < marker_pixels_down.size(); i++) {
//             marker_pixels_sorted.emplace_back(marker_pixels_down[i]);
//         }
//         for (int i = 0;  i < marker_pixels_sorted.size(); i++){
//             std::cout << "[roi process] pixel xy: " << marker_pixels_sorted[i].x << ", " << marker_pixels_sorted[i].y << std::endl;
//         }
//         pointsVector.clear();
// //        pointsVector = marker_pixels_sorted; //改用下面refine
//         //再采用灰度图去refine坐标
//         if(!refine_pixel(marker_pixels_sorted,pointsVector,ir_img)){
//             printf(RED"refine_pixel failed! use origin points.\n" RESET);
//             pointsVector.clear();
//             pointsVector = marker_pixels_sorted;
//         }
//         return true;

}

bool PnPTargetNodeROS::extractFeatures(cv::Mat &frame, vector<cv::Point2f> &pointsVector){
    pointsVector.clear();
    cv::goodFeaturesToTrack(frame, pointsVector, landmark_num, 0.01, 10);

#ifdef USE_4_Point
    sort(pointsVector.begin(), pointsVector.end(), [=](cv::Point pt1, cv::Point pt2){return  pt1.x < pt2.x;});
    std::vector<cv::Point2f> marker_pixels_left(pointsVector.begin(), pointsVector.begin() + 2);
    std::vector<cv::Point2f> marker_pixels_right(pointsVector.begin() + 3, pointsVector.begin() + 5);
    sort(marker_pixels_left.begin(), marker_pixels_left.end(), [=](cv::Point pt1, cv::Point pt2){return  pt1.y < pt2.y;});
    sort(marker_pixels_right.begin(), marker_pixels_right.end(), [=](cv::Point pt1, cv::Point pt2){return  pt1.y < pt2.y;});
    marker_pixels_sorted.clear();
    //使用上面的点
    marker_pixels_sorted.emplace_back(marker_pixels_left[0]);
    marker_pixels_sorted.emplace_back(marker_pixels_right[0]);
    //使用下面的点
    marker_pixels_sorted.emplace_back(marker_pixels_left[1]);
    marker_pixels_sorted.emplace_back(marker_pixels_right[1]);
    //填入中间的IR灯的点
    marker_pixels_sorted.emplace_back(pointsVector[2]);

    //  for (int i = 0;  i < marker_pixels_sorted.size(); i++){
    //    std::cout << "[roi process] pixel xy: " << marker_pixels_sorted[i].x << ", " << marker_pixels_sorted[i].y << std::endl;
    //  }
    pointsVector.clear();
    pointsVector = marker_pixels_sorted;

    //use the distance between landmarks to check
    double x_10 = pointsVector[1].x - pointsVector[0].x;
    double x_32 = pointsVector[3].x - pointsVector[2].x;
    double y_10 = pointsVector[1].y - pointsVector[0].y;
    double y_32 = pointsVector[3].y - pointsVector[2].y;
    if(abs(x_10 - x_32) > 10 || abs(y_10 - y_32) > 10){
        ROS_ERROR(BOLDRED "[Geometry Check False] x_10: %f, x_32: %f, y_10: %f, y_32: %f" RESET, x_10, x_32, y_10, y_32);
        return false;
    }else{
        return true;
    }
#endif

}

/**
 * @brief optical flow the given points
 * @param frame
 * @param outputPointsVector
 * @return
 */
bool PnPTargetNodeROS::optical_flow(cv::Mat &frame, vector<cv::Point2f> &pointsVector)
{
//    ROS_WARN("optical_flow function");
    //检查图像是否为空
    if (frame.empty()){
        ROS_ERROR("optical_follow ,NO frame !!!");
        return false;
    }
    
    nextImg = frame;

//    cv::imshow("nextImg",nextImg);
//    cv::waitKey(1);

    //尝试先用Fast去提取点，如果错误，然后再用光流跟踪
    if(extractFeatures(nextImg, nextImgPts)){
        cv::Rect rect_temp;
        rect_temp.x = (int)nextImgPts[0].x;
        rect_temp.y = (int)nextImgPts[0].y;
#ifdef USE_4_Point
      rect_temp.width = (int)nextImgPts[1].x - (int)nextImgPts[0].x;
      rect_temp.height = (int)nextImgPts[2].y - (int)nextImgPts[0].y;
      opti_trust_region.x = max(rect_temp.x - (int)(0.5 * rect_temp.width), 0);
      opti_trust_region.y = max(rect_temp.y - (int)(0.5 * rect_temp.height), 0);
      opti_trust_region.width = min((int)(rect_temp.width * 2), 640 - opti_trust_region.x);
      opti_trust_region.height = min((int)(rect_temp.height * 2), 480 - opti_trust_region.y);
#endif
//        printf("[trust_region after opti_flow] rect_temp.x = %d, .y = %d, width = %d, height = %d\n", rect_temp.x, rect_temp.y, rect_temp.width, rect_temp.height);
        printf("[opti_trust_region after opti_flow] .x = %d, .y = %d, width = %d, height = %d\n", opti_trust_region.x, opti_trust_region.y, opti_trust_region.width, opti_trust_region.height);
        //circle
        cv::Point2f top_left(opti_trust_region.x, opti_trust_region.y);
        cv::Point2f top_right(min(opti_trust_region.x+opti_trust_region.width, 640), opti_trust_region.y);
        cv::Point2f bottom_left(opti_trust_region.x, min(opti_trust_region.y+opti_trust_region.height,480));
        cv::Point2f bottom_right(min(opti_trust_region.x+opti_trust_region.width,640), min(opti_trust_region.y+opti_trust_region.height,480));
        cv::circle(ir_img_color_show,top_left,3,cv::Scalar(0,255,0),-1);
        cv::circle(ir_img_color_show,top_right,3,cv::Scalar(0,255,0),-1);
        cv::circle(ir_img_color_show,bottom_left,3,cv::Scalar(0,255,0),-1);
        cv::circle(ir_img_color_show,bottom_right,3,cv::Scalar(0,255,0),-1);
        cv::line(ir_img_color_show, top_left, top_right, cv::Scalar(255,255,0), 1);
        cv::line(ir_img_color_show, top_right, bottom_right, cv::Scalar(255,255,0), 1);
        cv::line(ir_img_color_show, bottom_right, bottom_left, cv::Scalar(255,255,0), 1);
        cv::line(ir_img_color_show, bottom_left, top_left, cv::Scalar(255,255,0), 1);
        printf("[optical_flow] by extractFeatures get %zu contours\n", nextImgPts.size());
        pointsVector.clear();
        pointsVector = nextImgPts;
        //交换前后帧图像和特征点位置
        swap(nextImg, prevImg);
        swap(prevImgPts, nextImgPts);
        printf("Fast Feature Extraction Success!\n");
        return true;
    }

    //构造前一张图像和光点
    printf("nextImg.size = %d, %d\n", nextImg.rows, nextImg.cols);
//    nextImg = frame; //旧代码，不需要mask
    prevImgPts = pointsVector;
    nextImgPts.clear();
    nextImgPts.resize(prevImgPts.size());
    status.clear();//光流跟踪状态
    err.clear();
    //检查光点数是否正确
    if (prevImgPts.size() != landmark_num){
        swap(nextImg, prevImg); //图像迭代成最新的，然后结束
        ROS_ERROR("prevImgPts.size() != 6 but == %d", int(prevImgPts.size()));
        return false;
    }
    //检查前一张图像是否存在
    if (prevImg.empty()){
        nextImg.copyTo(prevImg);
        ROS_INFO("prevImg is empty ,so nextImg copy to prevImg");
    }
//    cv::imshow("nextImg",nextImg);
//    cv::imshow("trust_region_mask",trust_region_mask);
//    cv::waitKey(1);

    calcOpticalFlowPyrLK(prevImg, nextImg, prevImgPts, nextImgPts, status, err, winSize,
                         3, termcrit, 0, 0.001);
//    ROS_INFO("complete opticalFlow");
    size_t i, k;
    for (i = k = 0; i < nextImgPts.size(); i++) {
//        std::cout << nextImgPts[i] << std::endl;
        ///TODO why is status false
        if (!status[i]){
            ROS_ERROR("[%zu] optical follow status error !!", i);
            prevImgPts.clear();
            nextImgPts.clear();
            return false;
        }
        if(nextImgPts[i].x < 10 || nextImgPts[i].x > ir_img.cols-10 || nextImgPts[i].y < 10 || nextImgPts[i].y > ir_img.rows-10){
            ROS_ERROR("points edge: failed.");
            return false;
        }
//        nextImgPts[k++] = nextImgPts[i];// according to status, only assign detected element into nextImgPts
//        printf("[after opti_flow] nextImgPts[%zu].x = %f, .y = %f\n", i, nextImgPts[i].x, nextImgPts[i].y);
    }
    //光流之后也需要做检查，保证没有追踪错误，否则直接return false. 按行坐标[3]>[2]>[1]>[0],第二行[4]<[1]。按列坐标[4]>[0],[5]>[2]
  //use the distance between landmarks to check

#ifdef USE_4_Point
  //use the distance between landmarks to check
  double x_10 = pointsVector[1].x - pointsVector[0].x;
  double x_32 = pointsVector[3].x - pointsVector[2].x;
  double y_10 = pointsVector[1].y - pointsVector[0].y;
  double y_32 = pointsVector[3].y - pointsVector[2].y;
  if(abs(x_10 - x_32) > 10 || abs(y_10 - y_32) > 10){
    ROS_ERROR(BOLDRED "[Geometry Check False] x_10: %f, x_32: %f, y_10: %f, y_32: %f" RESET, x_10, x_32, y_10, y_32);
    return false;
  }
#endif

    if (prevImgPts.size() == nextImgPts.size()) {
        //TODO:加入pixel_refine函数，让点准确提取出来并且受噪声影响较小。
        pointsVector.clear();
        if(!refine_pixel(nextImgPts, pointsVector, nextImg)){
            printf(RED"refine_pixel failed! use origin points.\n" RESET);
            if(pointsVector.size() < 6){
                //说明没有全部的点都refine成功，很有可能是光流给进去的点不对，所以直接return false。此处正常来讲都可以refine成功
                return false;
            }
            pointsVector.clear();
            pointsVector = nextImgPts; //如果refine失败，则直接把 nextImgPts 给 pointsVector
        }

        //将trust_region重新置为光流追踪后的点的外延区域，trust_region以检测出的目标区域块为中心的3*3的9个区域块都为置信区域
        // 20231205 not use trust_region after optical flow
        cv::Rect rect_temp;
        rect_temp.x = (int)nextImgPts[0].x;
        rect_temp.y = (int)nextImgPts[0].y;
#ifdef USE_4_Point
        rect_temp.width = (int)nextImgPts[1].x - (int)nextImgPts[0].x;
        rect_temp.height = (int)nextImgPts[2].y - (int)nextImgPts[0].y;
        opti_trust_region.x = max(rect_temp.x - (int)(0.5 * rect_temp.width), 0);
        opti_trust_region.y = max(rect_temp.y - (int)(0.5 * rect_temp.height), 0);
        opti_trust_region.width = min((int)(rect_temp.width * 2), 640 - opti_trust_region.x);
        opti_trust_region.height = min((int)(rect_temp.height * 2), 480 - opti_trust_region.y);
#endif
//        printf("[trust_region after opti_flow] rect_temp.x = %d, .y = %d, width = %d, height = %d\n", rect_temp.x, rect_temp.y, rect_temp.width, rect_temp.height);
        printf("[opti_trust_region after opti_flow] .x = %d, .y = %d, width = %d, height = %d\n", opti_trust_region.x, opti_trust_region.y, opti_trust_region.width, opti_trust_region.height);
        //circle
        cv::circle(ir_img_color_show,cv::Point2f(opti_trust_region.x, opti_trust_region.y),2,cv::Scalar(0,255,0),-1);
        cv::circle(ir_img_color_show,cv::Point2f(min(opti_trust_region.x+opti_trust_region.width, 640), opti_trust_region.y),2,cv::Scalar(0,255,0),-1);
        cv::circle(ir_img_color_show,cv::Point2f(opti_trust_region.x, min(opti_trust_region.y+opti_trust_region.height,480)),2,cv::Scalar(0,255,0),-1);
        cv::circle(ir_img_color_show,cv::Point2f(min(opti_trust_region.x+opti_trust_region.width,640), min(opti_trust_region.y+opti_trust_region.height,480)),2,cv::Scalar(0,255,0),-1);
        //交换前后帧图像和特征点位置
        swap(nextImg, prevImg);
        swap(prevImgPts, nextImgPts);
        return true;
    }else {
        swap(nextImg, prevImg);
        ROS_ERROR("prevImgPts.size() == nextImgPts.size() failed. We try Fast to detect corner points.");
        //如果光流失败尝试用FAST再次检测角点
//        vector<cv::Point2f> pointsVectorByExtractFeat;
        return false;
    }
}

bool PnPTargetNodeROS::pnp_process(vector<cv::Point2f> &pointsVector){
    printf("pnp_process function, opticalGoodFlag = %d, roiGoodFlag = %d\n",opticalGoodFlag, roiGoodFlag);

#ifdef USE_4_Point
  //use the distance between landmarks to check
  double x_10 = pointsVector[1].x - pointsVector[0].x;
  double x_32 = pointsVector[3].x - pointsVector[2].x;
  double y_10 = pointsVector[1].y - pointsVector[0].y;
  double y_32 = pointsVector[3].y - pointsVector[2].y;
  if(abs(x_10 - x_32) > 10 || abs(y_10 - y_32) > 10){
    ROS_ERROR(BOLDRED "[Geometry Check False] x_10: %f, x_32: %f, y_10: %f, y_32: %f" RESET, x_10, x_32, y_10, y_32);
    return false;
  }
#endif

    //solvePnP
    solvePnP(drone_landmarks_cv, pointsVector, cameraMatrix, distCoeffs, outputRvecRaw, outputTvecRaw, false, cv::SOLVEPNP_EPNP);
    Eigen::Vector3d eulerAngles;
    getEulerAngles(outputRvecRaw,eulerAngles, target_q_in_img);
    target_pos_in_img << outputTvecRaw.val[0], outputTvecRaw.val[1], outputTvecRaw.val[2];
    printf(YELLOW "[PnP Solve target] x: %.3f, y: %.3f, z: %.3f\n" RESET, target_pos_in_img[0], target_pos_in_img[1], target_pos_in_img[2]);
    msg_target_pose_from_img.header.stamp = stamp;
    target_pos_in_img[0] = min(max(target_pos_in_img[0], -2.0), 2.0);
    target_pos_in_img[1] = min(max(target_pos_in_img[1], -2.0), 2.0);
    target_pos_in_img[2] = min(max(target_pos_in_img[2], 0.5), 4.0);
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
    if(abs(msg_target_pose_from_img.pose.position.x) > 2 || abs(msg_target_pose_from_img.pose.position.y) > 2 || abs(msg_target_pose_from_img.pose.position.z) > 5){
        cv::putText(ir_img_color_show,
                    "out_range",
                    cv::Point(300, 140), cv::FONT_HERSHEY_TRIPLEX ,0.8,cv::Scalar(255,255,0),2,8,false);
        return false;
    }
    return true;
}

/**
 * @brief 从不同范围尺度和阈值计算marker_pixel的中心点
 * @param pts_raw
 * @return
 */
bool PnPTargetNodeROS::refine_pixel(std::vector<cv::Point2f> &pts_raw, std::vector<cv::Point2f> &pts_refine, cv::Mat &img){
//    cv::HoughCircles
    for(int i = 0; i < pts_raw.size(); i++){
        cv::rectangle(ir_img_color_show, cv::Point2f(pts_raw[i].x-win_width, pts_raw[i].y-win_width),
                      cv::Point2f(pts_raw[i].x+win_width, pts_raw[i].y+win_width),cv::Scalar(200,20,0),1);

        const int n = (2*win_width+1) * (2*win_width+1); //窗内最多这么多像素值会被加入计算
        Eigen::Matrix3Xi filterPts; // xy存储像素位置，z存储权重 3行n列
        filterPts.resize(3, n);
        int count_filter = 0;
        int weight_sum = 0; //权重总数
        int threshold = 232; //选取大于阈值的点用作中心计算
        //取均值思路，但为每个超过阈值而参与的点，根据灰度值赋予融合权重
        //若threshold初始取的不合适，则至少选出一个点再跳出
        while(1){
//            printf(GREEN "threshold = %d\n" RESET, threshold);
            filterPts.setZero();
            weight_sum = 0;
            count_filter = 0;
            for(int j = -win_width + 1; j < win_width; j++){
                for(int k = -win_width + 1; k < win_width; k++){
                    int index_x = j + pts_raw[i].x;
                    int index_y = k + pts_raw[i].y;
                    if(img.at<uchar>(index_y,index_x) > threshold){ //把阈值调整，让寻找中心更准确
                        filterPts(0, count_filter) = index_x;
                        filterPts(1, count_filter) = index_y;
                        filterPts(2, count_filter) = img.at<uchar>(index_y,index_x) - threshold; //xy位置，和超过阈值的灰度值(作为权重 )
                        weight_sum += filterPts(2, count_filter);
                        count_filter++;
                    }
                }
            }
//            printf(REDPURPLE "refine threshold now = %d\n" RESET, threshold);
            if(count_filter < 2){ //最少也要选出2个点来做权重融合，如果是1个或者0个就用之前的pts_raw
                threshold -= 1; //若阈值太高选不出点，则降低阈值
                if(threshold < 150){
                    printf(REDPURPLE "refine threshold too low = %d, return false ..., maybe optical flow track failed\n" RESET, threshold);
                    return false;
                }
            }else if(count_filter > 9){
                threshold += 1; //若阈值太低选出点过多，则增加阈值
                if(threshold == 255){
                    printf(REDPURPLE "refine threshold too high = %d, use all the points\n" RESET, threshold);
                    break;
                }
            }else{
//                printf(GREEN "refine count_filter = %d\n" RESET, count_filter);
                break;//选取数目合适，跳出循环
            }
        }
//        printf(GREEN "count_filter: %d, weight_sum: %d\n" RESET, count_filter, weight_sum);
        float u_pixel_weighted = 0;
        float v_pixel_weighted = 0;
        for (int j = 0; j < count_filter; ++j) {
            u_pixel_weighted += filterPts(0,j) * 1.0*filterPts(2,j)/weight_sum; //u pixel 乘上该像素权重在总权重中的占比
            v_pixel_weighted += filterPts(1,j) * 1.0*filterPts(2,j)/weight_sum; //v pixel 乘上该像素权重在总权重中的占比
//            printf(YELLOW "u: %d, v:%d, weight:%f\n" RESET,filterPts(0,j), filterPts(1,j), 1.0*filterPts(2,j)/weight_sum);
        }
        pts_raw[i].x = u_pixel_weighted;
        pts_raw[i].y = v_pixel_weighted;
//        printf(REDPURPLE "pts[%zu] u_pixel_weighted: %f, v_pixel_weighted: %f\n" RESET,i, u_pixel_weighted, v_pixel_weighted);
        pts_refine.emplace_back(pts_raw[i]);
    }
    return true;
}

/**
* 将欧拉角转化为四元数
* @param roll
* @param pitch
* @param yaw
* @return 返回四元数
*/
geometry_msgs::Quaternion PnPTargetNodeROS::euler2quaternion(float roll, float pitch, float yaw) {
    geometry_msgs::Quaternion temp;
    temp.w = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    temp.x = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    temp.y = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
    temp.z = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
    return temp;
}

Eigen::Quaterniond PnPTargetNodeROS::euler2quaternion_eigen(float roll, float pitch, float yaw) {
    Eigen::Quaterniond temp;
    temp.w() = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    temp.x() = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    temp.y() = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
    temp.z() = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
    return temp;
}

/**
* 将四元数转化为欧拉角形式
* @param x
* @param y
* @param z
* @param w
* @return 返回Vector3的欧拉角
*/
Eigen::Vector3d PnPTargetNodeROS::quaternion2euler(float x, float y, float z, float w) {
    Eigen::Vector3d temp;
    temp[0] = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    // I use ENU coordinate system , so I plus ' - '
    temp[1] = -asin(2.0 * (z * x - w * y));
    temp[2] = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return temp;
}

void PnPTargetNodeROS::getEulerAngles(cv::Vec3d &rvec, Eigen::Vector3d &eulerAngles, Eigen::Quaterniond &q){
    cv::Vec3d rvec_n = normalize(rvec);
    double n = norm(rvec);
    Eigen::AngleAxisd rotation_vector(n,Eigen::Vector3d(rvec_n[0],rvec_n[1],rvec_n[2]));
    Eigen::Matrix3d R;
//    std::cout << "n = " << n << "\trecv_n = " << rvec_n[0] << " " << rvec_n[0] << " " << rvec_n[0] << std::endl;
    R = rotation_vector.toRotationMatrix();
//    std::cout << "R = " << R.matrix() << std::endl;
    q = Eigen::Quaterniond(rotation_vector);
//    std::cout << "q = " << q.coeffs().transpose() << std::endl;
    eulerAngles = R.eulerAngles(2,1,0);
}