//
// Created by hzj on 24-3-28.
//
#include "multi_camera_cooperation/landmark_extraction_node_ros.h"


using namespace std;

LandmarkExtractionNodeROS::LandmarkExtractionNodeROS(){
    
}

void LandmarkExtractionNodeROS::init(ros::NodeHandle &nh){

//============== Read ros parameter =====================//
    nh.param<std::string>("cam", cam, "camA"); 
    nh.param<int>("landmark_num", landmark_num, 4); 
    nh.param<int>("ir_binary_threshold", ir_binary_threshold, 50);
    nh.param<std::string>("write_image_path", write_image_path, "default");  
//============== Read ros parameter =====================//
    
//============================= Initialize ROS topic =============================//
#ifdef IMG_COMPRESSED

    sub_ir_img = nh.subscribe("/ir_mono_"+cam, 1, &LandmarkExtractionNodeROS::ir_compressed_img_cb, this);
    ROS_INFO("we are using compressed image");
#else
    sub_ir_img = nh.subscribe("/ir_mono_"+cam, 1, &LandmarkExtractionNodeROS::ir_compressed_img_cb, this);
    // sub_ir_img = nh.subscribe("/ir_mono_"+cam, 1, &LandmarkExtractionNodeROS::ir_raw_img_cb, this);
    ROS_INFO("we are using raw image");
#endif
    // sub_ir_img = nh.subscribe("/ir_mono_"+cam, 1, &LandmarkExtractionNodeROS::ir_raw_img_cb, this);
#ifdef SHOW_ORIGIN_IMG
    pub_ir_img = nh.advertise<sensor_msgs::Image>("ir_mono/origin",1);
#endif
    pub_marker_pixel = nh.advertise<multi_camera_cooperation::landmark>("ir_mono/marker_pixel",10);
    pub_ir_show_img = nh.advertise<sensor_msgs::Image>("ir_mono/show",1);
    pub_ir_binary_img = nh.advertise<sensor_msgs::Image>("ir_mono/ir_binary",1);
    pub_ir_erode_dilate_img = nh.advertise<sensor_msgs::Image>("ir_mono/ir_erode_dilate",1);

    
    ROS_INFO("Publisher subscriber initialized");
    ROS_INFO("[SWARM_DETECT] Finish initialize swarm detector, wait for data...");

    cv_ptr_compressed_ir = boost::make_shared<cv_bridge::CvImage>();
    cv_ptr_raw_ir = boost::make_shared<cv_bridge::CvImage const>();
}   

//============================= ir_img receive =============================//
void LandmarkExtractionNodeROS::ir_raw_img_cb(const sensor_msgs::Image::ConstPtr &msg){
    ROS_INFO("ir_img_cb");
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
    landmark_pose_extract();
}

void LandmarkExtractionNodeROS::ir_compressed_img_cb(const sensor_msgs::CompressedImage::ConstPtr &msg){
    ROS_INFO("ir_img_cb");
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
    landmark_pose_extract();
}
//============================= ir_img receive =============================//

void LandmarkExtractionNodeROS::landmark_pose_extract(){
    ir_img_color_show = cv::Mat::zeros(ir_img.size(), CV_8UC3);
    cv::cvtColor(ir_img, ir_img_color_show, CV_GRAY2BGR);
    //光流追踪
    // if(opticalReadyFlag){
    //     opticalGoodFlag = optical_flow(ir_img,marker_pixels);
    // }
    //如果光流追踪失败，则重新清除marker,然后进行ROI
    if(!opticalGoodFlag){
        ROS_WARN("optical flow fails, try to roi detect!");
        marker_pixels.clear();
        roiGoodFlag = ir_img_process(ir_img, marker_pixels, 2);
    }

    if(opticalGoodFlag || roiGoodFlag){
        landmark_msg_generate(marker_pixels);
        pub_marker_pixel.publish(landmark_msg);
    }

#ifdef ENABLE_VISUALIZATION
    // // 可视化求解情况
    // if(pnpGoodFlag){
    //     cv::putText(ir_img_color_show,
    //                 "pnp",
    //                 cv::Point(470, 20), cv::FONT_HERSHEY_TRIPLEX ,0.65,cv::Scalar(0,255,0),1,false);
    // }else{
    //     cv::putText(ir_img_color_show,
    //                 "pnp",
    //                 cv::Point(470, 20), cv::FONT_HERSHEY_TRIPLEX ,0.65,cv::Scalar(0,0,255),1,false);
    // }

    // 画横线框，和图像中心点，校验求解目标原点是否正确
    // cv::circle(ir_img_color_show,cv::Point2f(320,240),2,cv::Scalar(255,0,0),1);
    // cv::line(ir_img_color_show,cv::Point2f(0,240),cv::Point2f(640,240),cv::Scalar(255,0,0),1);
    // cv::line(ir_img_color_show,cv::Point2f(320,0),cv::Point2f(320,480),cv::Scalar(255,0,0),1);
    // cv::namedWindow("ir_img_color_show", cv::WINDOW_NORMAL);
    // cv::resizeWindow("ir_img_color_show", 2560, 1920);
    // cv::imshow("ir_img_color_show", ir_img_color_show);
    // cv::waitKey(1);
#ifdef USE_4_Point
    if(marker_pixels.size() == landmark_num){
       Square_shape_identity(marker_pixels);
    }
#endif
    
    for (int i = 0; i < marker_pixels.size(); i++) { 
        cv::circle(ir_img_color_show, cv::Point2i(marker_pixels[i].x, marker_pixels[i].y), 5,  cv::Scalar(0,0,255), 1, cv::LINE_AA);
        cv::putText(ir_img_color_show, std::to_string(i), cv::Point2i(marker_pixels[i].x, marker_pixels[i].y), cv::FONT_HERSHEY_TRIPLEX, 0.65, cv::Scalar(0,255,0), 1, false);
    }
    auto ir_show_msg_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", ir_img_color_show).toImageMsg();
    pub_ir_show_img.publish(ir_show_msg_img);
#endif
}


bool LandmarkExtractionNodeROS::ir_img_process(cv::Mat &_ir_img, vector<cv::Point2f> &pointsVector, int model){
    ROS_WARN("ir_img_process function");
    if(_ir_img.empty())
    {
        ROS_ERROR("ir_img_process, NO _ir_img !!!");
        return false;
    }

    if(model == 1){//尝试用FAST检测特征点
        if(extractFeatures(ir_img, pointsVector)){
            printf("[ir_img_process] by extractFeatures get %zu contours\n", pointsVector.size());
            return true;
        }else{
            return false;
        }
    }

    if(model == 2){//尝试用binary检测特征点
        if(binary_threshold(ir_img, pointsVector)){
            printf("[ir_img_process] by binary_threshold get %zu contours\n", pointsVector.size());
            return true;
        }else{
            return false;
        }
    }

    ROS_ERROR("error process method, ir_img_process failed !!!");
    return false;

}

bool LandmarkExtractionNodeROS::extractFeatures(cv::Mat &frame, vector<cv::Point2f> &pointsVector){
    pointsVector.clear();
    cv::goodFeaturesToTrack(frame, pointsVector, landmark_num, 0.01, 10);
    if(pointsVector.size() != 0){
        return true;
    }else{
        return false;
    }
}

bool LandmarkExtractionNodeROS::binary_threshold(cv::Mat &frame, vector<cv::Point2f> &pointsVector){
    pointsVector.clear();

    int try_count = 0;


    double max_area = 5;
    while(1){
        //二值化及膨胀腐蚀
        threshold(frame, ir_binary, ir_binary_threshold, 255, cv::THRESH_BINARY);
        // cv::erode(ir_binary, ir_erode, erodeElement);
        ir_erode = ir_binary;
        auto ir_erode_dilate_img = cv_bridge::CvImage(std_msgs::Header(), "mono8", ir_erode).toImageMsg();
        pub_ir_erode_dilate_img.publish(ir_erode_dilate_img);
        //寻找轮廓
        findContours(ir_erode, ir_contours_unfiltered, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        // findContours(ir_erode, ir_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        // cout << "unfiltered_contours = " << ir_contours_unfiltered.size() << endl;
        // for (int i=0;i<ir_contours_unfiltered.size();i++){
        //     double area = cv::contourArea(ir_contours_unfiltered[i]);
        //     ROS_INFO("the area = %f", area);
        //     if(area < max_area){
        //         ir_contours.push_back(ir_contours_unfiltered[i]);
        //     }
        // }
        // cout<<"filtered_contours = "<<ir_contours.size()<<endl;
        
        if(ir_contours_unfiltered.size() < landmark_num){
            ir_binary_threshold -= 5;
            printf(YELLOW "[down] ir_binary_threshold set to %d\n", ir_binary_threshold);
            if(ir_binary_threshold <= 5){
                ir_binary_threshold = 80;
                return false;
            }
        }else if(ir_contours_unfiltered.size() > landmark_num){
            ir_binary_threshold += 5;
            printf(YELLOW "[up] ir_binary_threshold set to %d\n", ir_binary_threshold);
            if(ir_binary_threshold >= 220){
                ir_binary_threshold = 80;
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

    // filter the contours
    for (int i = 0; i < ir_contours_unfiltered.size(); i++){
            // double area = cv::contourArea(ir_contours_unfiltered[i]);
            // ROS_INFO("the area = %f", area);
            // if(area < max_area){
            //     ir_contours.push_back(ir_contours_unfiltered[i]);
            //   }

            ir_contours.push_back(ir_contours_unfiltered[i]);
    }
    ir_contours_unfiltered.clear();
    cout << "filtered_contours = " << ir_contours.size() << endl;
    //在ROI图像上画出边框,可视化发布边框
    cv::Mat ir_binary_color_show;
    cv::cvtColor(ir_erode, ir_binary_color_show, CV_GRAY2BGR);

    for (int i = 0; i < ir_contours.size(); i++) {
        cv::Rect bbox;
        bbox = boundingRect(ir_contours[i]);
        cv::rectangle(ir_binary_color_show, bbox, cv::Scalar(0, 255, 0), 5);}
    auto ir_binary_msg_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", ir_binary_color_show).toImageMsg();
    pub_ir_binary_img.publish(ir_binary_msg_img);

    drawContours(ir_img_color_show, ir_contours, -1, (0, 255, 0), 1);

    if (ir_contours.size() == landmark_num) {
        printf(GREEN "[ir_img process] get %zu contours\n", ir_contours.size());
        for (int i = 0; i < ir_contours.size(); i++) {
            cv::Rect bbox;
            bbox = boundingRect(ir_contours[i]);
            pointsVector.emplace_back(cv::Point2f((bbox.tl() + bbox.br()).x / 2.0, (bbox.tl() + bbox.br()).y / 2.0));
            std::cout << "pointsVector xy = " << pointsVector[i].x << ", " << pointsVector[i].y << std::endl;
        }
    }
    ir_contours.clear();
    //再采用灰度图去refine坐标
    // if(!refine_pixel(marker_pixels_sorted,pointsVector,ir_img)){
    //     printf(RED"refine_pixel failed! use origin points.\n" RESET);
    //     pointsVector.clear();
    //     pointsVector = marker_pixels_sorted;
    // }

    if(pointsVector.size() != 0){
        return true;
    }else{
        return false;
    }
}

/**
 * @brief optical flow the given points
 * @param frame
 * @param outputPointsVector
 * @return
 */
bool LandmarkExtractionNodeROS::optical_flow(cv::Mat &frame, vector<cv::Point2f> &pointsVector)
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
// 解释一下这部分的内容，这里是根据光流追踪后的点，来确定一个更大的可信区域，这个可信区域是以光流追踪后的点为中心，然后向外扩展一定的区域
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

    if (prevImgPts.size() == nextImgPts.size()) {
        //TODO:加入pixel_refine函数，让点准确提取出来并且受噪声影响较小。
        pointsVector.clear();
        if(!refine_pixel(nextImgPts, pointsVector, nextImg)){
            printf(RED"refine_pixel failed! use origin points.\n" RESET);
            if(pointsVector.size() < 4){
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

/**
 * @brief 从不同范围尺度和阈值计算marker_pixel的中心点
 * @param pts_raw
 * @return
 */
bool LandmarkExtractionNodeROS::refine_pixel(std::vector<cv::Point2f> &pts_raw, std::vector<cv::Point2f> &pts_refine, cv::Mat &img){
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

void LandmarkExtractionNodeROS::landmark_msg_generate(vector<cv::Point2f> &pointsVector){  
    landmark_msg.header.stamp = ros::Time::now();  
    landmark_msg.header.frame_id = cam;  
 
    landmark_msg.x.resize(pointsVector.size());  
    landmark_msg.y.resize(pointsVector.size());  
    for (int i = 0; i < pointsVector.size(); i++)  
    {  
        landmark_msg.x[i] = pointsVector[i].x;  
        landmark_msg.y[i] = pointsVector[i].y;  
    }
}

bool LandmarkExtractionNodeROS::Square_shape_identity(vector<cv::Point2f> &pointsVector){

    marker_pixels_sorted.clear();
    marker_pixels_up.clear();
    marker_pixels_down.clear();

    bool SquareShapeGoodFlag = true;
    
    if (pointsVector.size() != 4)
    {
        return false;
    }

//     // we choose to sort the points by their y coordinate first, then by their x coordinate
//    sort(pointsVector.begin(), pointsVector.end(), [](const cv::Point2f& pt1, const cv::Point2f& pt2) {
//         return pt1.y < pt2.y;
//     });

//     std::vector<cv::Point2f> marker_pixels_up(pointsVector.begin(), pointsVector.begin() + 2);
//     std::vector<cv::Point2f> marker_pixels_down(pointsVector.begin() + 2, pointsVector.end());

//     sort(marker_pixels_up.begin(), marker_pixels_up.end(), [](const cv::Point2f& pt1, const cv::Point2f& pt2) {
//         return pt1.x > pt2.x;
//     });

//     sort(marker_pixels_down.begin(), marker_pixels_down.end(), [](const cv::Point2f& pt1, const cv::Point2f& pt2) {
//         return pt1.x > pt2.x;
//     });



//     marker_pixels_sorted.push_back(marker_pixels_up[0]);
//     marker_pixels_sorted.push_back(marker_pixels_down[0]);
//     marker_pixels_sorted.push_back(marker_pixels_down[1]); 
//     marker_pixels_sorted.push_back(marker_pixels_up[1]);


    // we choose to sort the points by their x coordinate first, then by their y coordinate

    sort(pointsVector.begin(), pointsVector.end(), [](const cv::Point2f& pt1, const cv::Point2f& pt2) {
        return pt1.x < pt2.x;
    });

    std::vector<cv::Point2f> marker_pixels_left(pointsVector.begin(), pointsVector.begin() + 2);
    std::vector<cv::Point2f> marker_pixels_right(pointsVector.begin() + 2, pointsVector.end());

    sort(marker_pixels_left.begin(), marker_pixels_left.end(), [](const cv::Point2f& pt1, const cv::Point2f& pt2) {
        return pt1.y < pt2.y;
    });

    sort(marker_pixels_right.begin(), marker_pixels_right.end(), [](const cv::Point2f& pt1, const cv::Point2f& pt2) {
        return pt1.y < pt2.y;
    });

    marker_pixels_sorted.push_back(marker_pixels_right[0]);
    marker_pixels_sorted.push_back(marker_pixels_right[1]);
    marker_pixels_sorted.push_back(marker_pixels_left[1]);
    marker_pixels_sorted.push_back(marker_pixels_left[0]);


    // 清空原始向量，并用排序后的点填充它


    // // check whether the edegs are parallel
    // Eigen::Vector2d linkvector0 = subtractPoints(marker_pixels_sorted[0], marker_pixels_sorted[1]);
    // Eigen::Vector2d linkvector1 = subtractPoints(marker_pixels_sorted[1], marker_pixels_sorted[2]);
    // Eigen::Vector2d linkvector2 = subtractPoints(marker_pixels_sorted[2], marker_pixels_sorted[3]);
    // Eigen::Vector2d linkvector3 = subtractPoints(marker_pixels_sorted[3], marker_pixels_sorted[0]);

    // if((abs(vectorAngle(linkvector0, linkvector2, 1) - 180) > 20) || (abs(vectorAngle(linkvector1, linkvector3, 1) - 180) > 20)){
    //     return false;
    // }


    pointsVector.clear();
    pointsVector = marker_pixels_sorted;

    // std::cout<<pointsVector<<std::endl;
    return true;
    

}