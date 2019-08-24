#include "MathTools.h"

float dist2D(cv::Point2f a, cv::Point2f b) {
	return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
}

double cal_angle(cv::Point2f a, cv::Point2f b) {
	float dot1 = a.x*b.x + a.y*b.y;
	float dot2 = dist2D(a, cv::Point(0, 0)) * dist2D(b, cv::Point(0, 0));
	return acos(dot1 / dot2) / CV_PI * 180;
}