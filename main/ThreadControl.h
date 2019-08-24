/**********************************************************
* Copyright (C) 2018 TYUT_TRoMaC 690208412@qq.com	      *
*                                                         *
*  	Write functions for each thread						  *
*                                                         *
*  @file     ThreadControl.h                              *
*  @brief                                                 *
*  Details.                                               *
*                                                         *
*  @author   Shuyuan.Yu                                   *
*  @email    690208412@qq.com                             *
*  @version  1.0                                          *
*  @date     2018/12/31                                   *
*  @license  GNU General Public License (GPL)             *
*---------------------------------------------------------*
*  Remark:                                                *
*---------------------------------------------------------*
*  Change History:                                        *
*  <Date>    | <Version> |  <Author>  | <Description>     *
*  2018/12/31|    1.0.   | Shuyuan.Yu | Create file       *
***********************************************************/
#pragma once
#include "thread"
#include "atomic"

#include "../camera/camera.h"
#include "../uart/serial.hpp"
#include "../detector/ArmorDetector.h"
#include "../fusion/CoordinatesFusion.h"
#include "../predictor/Predictor.h"
#include "../decision/Decision.h"

class processer{
public:
	processer();
	~processer();

	/**
	* @函数: ArmorDetectorThreadLoop
	* @描述: 装甲检测+预测+串口发送线程
	 */
	void ArmorDetectorThreadLoop();

	/**
	* @函数: InfoReadThreadLoop
	* @描述: 串口读取线程
	 */
	void InfoReadThreadLoop();

};