/**********************************************************
* Copyright (C) 2019 TYUT_TRoMaC 690208412@qq.com	      *
*                                                         *
*  	Write functions for each thread						  *
*                                                         *
*  @file     CoordiantesFusion.h                          *
*  @brief                                                 *
*  Details.                                               *
*                                                         *
*  @author   Shuyuan.Yu                                   *
*  @email    690208412@qq.com                             *
*  @version  1.0                                          *
*  @date     2019/3/19                                    *
*  @license  GNU General Public License (GPL)             *
*---------------------------------------------------------*
*  Remark:                                                *
*---------------------------------------------------------*
*  Change History:                                        *
*  <Date>    | <Version> |  <Author>  | <Description>     *
*  2019/3/19|    1.0.   | Shuyuan.Yu | Create file        *
***********************************************************/
#pragma once
#define PostProcesser_H
#ifdef PostProcesser_H
#define Predict
//#define None_Pose

#include "iostream"
#include "thread"
#include "mutex"
#include "atomic"
#include "opencv2/opencv.hpp"
#include "../detector/ArmorDetector.h"

class GimblaPose {
public:
	float yaw;
	short pitch;
	float timestamp;
};

class CoordinatesFusion {
public:
	CoordinatesFusion();
	/**
	* @函数: CoordinatesFusion
	* @描述: 初始化
	* @输入: short PMIN 编码值pitch的下限
			 short PC   编码值pitch的中心
			 short PMAX 编码值pitch的上限
	 */
	CoordinatesFusion(short PMIN , short PC,short PMAX);
	
	~CoordinatesFusion();
	
	/**
	* @函数: pushGimblaPose
	* @描述: 返回主控云台值
	* @输入: float yaw	   主控返回yaw编码值
			 short pitch   主控返回pitch编码值
			 float timestamp 主控返回的系统时间
	 */
	void pushGimblaPose(float yaw, short pitch, float timestamp);
	
	/**
	* @函数: caculateTargetPose
	* @描述: 得到目标的世界坐标
	* @输入: Point2f angle			敌方与摄像头成像平面中垂线的角度（x，y）
			 cv::Mat tvecDeviation  枪管到摄像头的平移矩阵
			 float distance			敌方距离
			 double timestamp		此时的时间
	* @输入输出:cv::Point2f &targetPose 敌方世界坐标
	* @返回: bool 成功转换
	*/
	bool caculateTargetPose(cv::Point2f angle, cv::Mat tvecDeviation,  float distance, double timestamp, cv::Point2f &targetPose);
	
	/**
	* @函数: getLatestData
	* @描述: 得到最后一次云台编码值
	* @返回: pair<float,short> 此刻的yaw，pitch
	*/
	pair<float,short> getLatestData();
	
	/**
	* @函数: getLatestData
	* @描述: 清除历史数据
	*/
	void clear();

private:
	/**
	* @函数: interpolateRobotPose
	* @描述: 根据时间对云台值进行插值
	* @输入: double timestamp 此刻时间
			 GimblaPose &pose 云台状态
	*/
	bool interpolateRobotPose(double timestamp, GimblaPose &pose);

private:
	double PoseDataSaveTime;//云台编码值的保存时间
	std::vector<GimblaPose> historyGimblaPose; //历史云台数据
	std::atomic<double> LatestGimblaPoseTime; //最后一次返回云台的时间

	short pitchMin; //pitch最小值
	short pitchCenter; //pitch中心值
	short pitchMax; //pitch最大值
};
#endif
