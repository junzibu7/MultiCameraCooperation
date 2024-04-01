//
// Created by hzj on 24-2-28.
//
#include "multi_camera_cooperation/math_tools.h"


/**
* 将欧拉角转化为四元数
* @param roll
* @param pitch
* @param yaw
* @return 返回四元数
*/
geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw) {
    geometry_msgs::Quaternion temp;
    temp.w = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    temp.x = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    temp.y = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
    temp.z = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
    return temp;
}

Eigen::Quaterniond euler2quaternion_eigen(float roll, float pitch, float yaw) {
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
Eigen::Vector3d quaternion2euler(float x, float y, float z, float w) {
    Eigen::Vector3d temp;
    temp[0] = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    // I use ENU coordinate system , so I plus ' - '
    temp[1] = -asin(2.0 * (z * x - w * y));
    temp[2] = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return temp;
}

void getEulerAngles(cv::Vec3d &rvec, Eigen::Vector3d &eulerAngles, Eigen::Quaterniond &q){
    cv::Vec3d rvec_n = normalize(rvec);
    double n = norm(rvec);
    Eigen::AngleAxisd rotation_vector(n,Eigen::Vector3d(rvec_n[0],rvec_n[1],rvec_n[2]));
    Eigen::Matrix3d R;
//    cout << "n = " << n << "\trecv_n = " << rvec_n[0] << " " << rvec_n[0] << " " << rvec_n[0] << endl;
    R = rotation_vector.toRotationMatrix();
//    cout << "R = " << R.matrix() << endl;
    q = Eigen::Quaterniond(rotation_vector);
//    cout << "q = " << q.coeffs().transpose() << endl;
    eulerAngles = R.eulerAngles(2,1,0);
}

float get_lines_arctan(float line_1_k, float line_2_k, int method)
{
    if (method == 0)
    {
        float tan_k = 0; //直线夹角正切值
        float lines_arctan;//直线斜率的反正切值
        tan_k = (line_2_k - line_1_k) / (1 + line_2_k*line_1_k); //求直线夹角的公式
        lines_arctan = atan(tan_k);
        return lines_arctan;
    }
    else
    {
        float tan_k = 0; //直线夹角正切值
        float lines_arctan;//直线斜率的反正切值
        tan_k = (line_2_k - line_1_k) / (1 + line_2_k*line_1_k); //求直线夹角的公式
        lines_arctan = atan(tan_k)* 180.0 / 3.1415926;

        return lines_arctan;
    }
}

// 计算向量的模  
double vectorNorm2D(Eigen::Vector2d& vec) {  
    return std::sqrt(vec(0)*vec(0) + vec(1)*vec(1)); 
}  
  
// 计算向量的点积  
double vectorDotProduct(Eigen::Vector2d& vec1, Eigen::Vector2d& vec2) {   
    return (vec1(0) * vec2(0)) + (vec1(1) * vec2(1)); // 分别计算x分量和y分量的点积，然后相加  
}
  
// 计算两个向量的夹角 
double vectorAngle(Eigen::Vector2d& vec1, Eigen::Vector2d& vec2, int method) {  
    double dotProduct = vectorDotProduct(vec1, vec2);  
    double norm1 = vectorNorm2D(vec1);  
    double norm2 = vectorNorm2D(vec2);  
  
    // 检查分母是否为零  
    if (norm1 * norm2 == 0.0) {  
        return 0.0;  // 或者返回一个错误码或抛出异常  
    }  
  
    // 使用acos函数计算夹角 
    if (method == 0) {
        return acos(dotProduct / (norm1 * norm2));//以弧度为单位
    }else if(method == 1){
        return acos(dotProduct / (norm1 * norm2)) * 180.0 / 3.1415926;
    }
      
}

bool checkRotationDirection(Eigen::Vector2d& init, Eigen::Vector2d& final) {  
    int crossProduct = init(0) * final(1) - init(1) * final(0);  
      
    if (crossProduct > 0) {  
        return true; //CLOCKWISE;  
    } else if (crossProduct < 0) {  
        return false; //COUNTER_CLOCKWISE;  
    } else {  
        return false; //COLLINEAR;  
    }  
}

Eigen::Vector2d subtractPoints(cv::Point2f& point1, cv::Point2f& point2) {   
    Eigen::Vector2d temp = Eigen::Vector2d(point2.x - point1.x, point2.y - point1.y);
    return temp;
} 


tf::Quaternion EigenQuaterniondToTFQuaternion(Eigen::Quaterniond q_EIGEN) 
{
    tf::Quaternion temp;
    temp.setX(q_EIGEN.x());
    temp.setY(q_EIGEN.y());
    temp.setZ(q_EIGEN.z());
    temp.setW(q_EIGEN.w());
    return temp;
} 

tf::Vector3 EigenVector3dToTFVector3(Eigen::Vector3d t)
{
    tf::Vector3 temp;
    temp[0] = t[0];
    temp[1] = t[1];
    temp[2] = t[2];
    return temp;
}

Eigen::Quaterniond TFQuaternionToEigenQuaterniond(tf::Quaternion q)
{
    Eigen::Quaterniond temp;
    temp.x() = q.x();
    temp.y() = q.y();
    temp.z() = q.z();
    temp.w() = q.w();
    return temp;
}

Eigen::Vector3d TFVector3ToEigenVector3d(tf::Vector3 t)
{
    Eigen::Vector3d temp;
    temp[0] = t[0];
    temp[1] = t[1];
    temp[2] = t[2];
    return temp;
}