/**********************************************************
* Copyright (C) 2018 TYUT_TRoMaC 690208412@qq.com	      *
*                                                         *
*  	preidict next coordinates   						  *
*                                                         *
*  @file     Predictor.h                                  *
*  @brief                                                 *
*  Details.                                               *
*                                                         *
*  @author   Shuyuan.Yu                                   *
*  @email    690208412@qq.com                             *
*  @date     2019/3/19                                    *
*  @license  GNU General Public License (GPL)             *
*---------------------------------------------------------*
*  Remark:                                                *
*---------------------------------------------------------*
*  Change History:                                        *
*  <Date>    | <Version> |  <Author>  | <Description>     *
*  2018/3/19|    1.0.   | Shuyuan.Yu | Create file        *
***********************************************************/

#pragma once
#include "iostream"
#include "math.h"
#include "opencv2/opencv.hpp"
#include "KalmanFilter.h"
#include "thread"
#include "atomic"

using namespace std;
using namespace cv;


class Predictor{
public:
	/**
	* @函数: Predictor
	* @描述: 初始化预判函数
	* @输入: float pC 表示pitch水平编码值
	 */
	Predictor(float pC);
	~Predictor();
public:
	/**
	* @函数: predict
    * @描述: 对绝对坐标进行预判
	* @输入: float yaw 装甲板所在的yaw角度值
			 float pitch 装甲板所在的pitch编码值
			 float yaw_now 此刻云台的yaw
			 float pitch_now 此刻云台的pitch
			 float distance , int bulletSpeed
	* @返回：pair<cv::Point2f,bool> Point2f预测后云台的(yaw，pitch) bool是否能够射击
	 */
	pair<cv::Point2f,bool> predict(float yaw, float pitch, float yaw_now, float pitch_now , float distance , int bulletSpeed);

private:
	/**
	* @函数: kalmanPredict
	* @描述: 对绝对坐标进行预判
	* @输入: float yaw 装甲板所在的yaw角度值
			 float pitch 装甲板所在的pitch编码值
			 float time 预判时长
	* @返回：Point2f 预测后云台的yaw，pitch
	*/
	cv::Point2f kalmanPredict(float yaw, float pitch, float time);
	/**
	* @函数: compensaBallisticDrop
	* @描述: 对弹道下坠的补偿（未考虑空气阻力）
	* @输入: float input 输入的pitch编码值
			 float distance 装甲板距离
			 float bulletSpeed 子弹速度
	* @返回：float 预测后云台的pitch
	*/
	float compensaBallisticDrop(float input, float distance, int bulletSpeed);
private:
	EigenKalman::KalmanFilter KF;
	Eigen::VectorXd x;
	float pitchCenter;//初始化pitch水平的编码值
public:
	atomic<int> shootSpeed;//发射速度 单位;米/秒
};