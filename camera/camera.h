#pragma once
#include "iostream"
#include "opencv2/opencv.hpp"

using namespace std;

class MyCamera {
public:
	MyCamera();
	~MyCamera();
	/**
	* @函数: open
	* @描述: 打开摄像头
	*/
	bool open();

	/**
	* @函数: getFrame
	* @描述: 取一张图片
	* @输入输出: cv::Mat & img 彩色图片
	*/
	bool getFrame(cv::Mat & img);

	void restart();

public:
	/**
	* @函数: loadMatrix
	* @描述: 读取摄像头参数
	* @输入: string filename 文件路径名称
	* @返回：成功读取？
	*/
	bool loadMatrix(string filename);

	/**
	* @函数: loadMatrix
	* @描述: 读取摄像头参数
	* @返回：图像畸变中心
	*/
	cv::Point2f picCenter();

	/**
	* @函数: pix2angle
	* @描述: 将像素转换为与摄像头畸变中心垂线的角度
	* @输入：cv::Point2f point像素值
			 bool shoot42 枪管角度矫正是42mm或者17mm
	* @输出: cv::Point2f &angle角度
	*/
	void pix2angle(cv::Point2f point, cv::Point2f &angle, bool shoot42);
	cv::Point2f pix2angle(cv::Point2f point, bool shoot42);

	/**
	* @函数: undistortPoints
	* @描述: 对点进行去畸变
	* @输入: cv::Point2f inputPoint 输入点
	* @输入输出：cv::Point2f & outputPoint 进行去畸变后像素点位置
	*/
	void undistortPoints(cv::Point2f inputPoint, cv::Point2f & outputPoint);
	cv::Point2f undistortPoints(cv::Point2f point);

private:
	cv::VideoCapture cam;
private:
	bool loadedMatrix = false;//加载相机参数的标志
	cv::Mat cameraMatrix;//相机内参
	cv::Mat distCoeffs;//相机畸变参数
};