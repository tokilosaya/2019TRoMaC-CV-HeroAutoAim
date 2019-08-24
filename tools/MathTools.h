#include "iostream"
#include "opencv2/opencv.hpp"
#include "math.h"

using namespace std;
using namespace cv;

float dist2D(cv::Point2f a, cv::Point2f b);
double cal_angle(cv::Point2f a, cv::Point2f b);