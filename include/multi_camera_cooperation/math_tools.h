#ifndef MATH_TOOLS_H
#define MATH_TOOLS_H

#include <cstdlib>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <cstdio>
#include <fstream>
#include <cmath>
#include <ctime>
#include <queue>
#include <vector>
#include <chrono>
#include <sstream>

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;


void getEulerAngles(cv::Vec3d &rvec, Eigen::Vector3d &eulerAngles, Eigen::Quaterniond &q);
float get_lines_arctan(float line_1_k, float line_2_k, int aaa);
double vectorNorm2D(Eigen::Vector2d& vec) ;
double vectorDotProduct(Eigen::Vector2d& vec1, Eigen::Vector2d& vec2);
double vectorAngle(Eigen::Vector2d& vec1, Eigen::Vector2d& vec2, int method);
Eigen::Vector2d subtractPoints(cv::Point2f& point1, cv::Point2f& point2);

#endif