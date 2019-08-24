/**********************************************************
* Copyright (C) 2019 TYUT_TRoMaC 690208412@qq.com	      *
*                                                         *
*  decide how to shoot 				                      *
*                                                         *
*  @file    Decision.h                                    *
*  @brief                                                 *
*  Details.                                               *
*                                                         *
*  @author   Shuyuan.Yu                                   *
*  @email    690208412@qq.com                             *
*  @date     2019/6/22                                    *
*  @license  GNU General Public License (GPL)             *
*---------------------------------------------------------*
*  Remark:                                                *
*---------------------------------------------------------*
*  Change History:                                        *
*  <Date>    | <Version> |  <Author>  | <Description>     *
*  2019/6/22|    1.0.   | Shuyuan.Yu | Create file        *
***********************************************************/
#pragma once
#include "iostream"
#include "opencv2/opencv.hpp"
#include "thread"
#include "atomic"


class Decision
{
public:
	Decision();
	~Decision();

public:
	/**
	* @函数: setState
	* @描述: 设置热量状态
	* @输入: int m 设置模式
			 int h42 42mm剩余热量
			 int h17 17mm剩余热量
			 int Speed 17mm射速
	* @返回:找到的灯柱数量
	*/
	void setState(int m, int h42, int h17, int Speed);

	/**
	* @函数: make
	* @描述: 决定打哪种子弹
	* @返回:tuple<cv::Mat, int, bool> Mat摄像头便宜枪管的平移矩阵，int射速，bool大或小枪管
	*/
	std::tuple<cv::Mat, int, bool> make();

private:
	std::atomic<int> mode;//mode 1:自动切换 17：只打小子弹 42：只打大子弹
	std::atomic<int> heat42;//42mm剩余热量
	std::atomic<int> heat17;//17mm剩余热量
	std::atomic<int> speed17;//17mm射速

	cv::Mat tvec42;//摄像头偏移大枪管的矩阵
	cv::Mat tvec17;//摄像头便宜小强管的矩阵
};

