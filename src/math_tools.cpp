#include "multi_camera_cooperation/math_tools.h"



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

Eigen::Vector2d subtractPoints(cv::Point2f& point1, cv::Point2f& point2) {   
    Eigen::Vector2d temp = Eigen::Vector2d(point2.x - point1.x, point2.y - point1.y);
    return temp;
} 

