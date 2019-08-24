/**********************************************************
* Copyright (C) 2018 Shuyuan.Yu  690208412@qq.com	      *
*                                                         *
*	detecte RM_Armor			                          *
*                                                         *
*  @file     ArmorDetector.h                              *
*  @brief                                                 *
*  Details.                                               *
*                                                         *
*  @author   Shuyuan.Yu                                   *
*  @email    690208412@qq.com                             *
*  @version  1.0                                          *
*  @date     2018/10/26                                   *
*  @license  GNU General Public License (GPL)             *
*---------------------------------------------------------*
*  Remark:                                                *
*---------------------------------------------------------*
*  Change History:                                        *
*  <Date>    | <Version> |  <Author>  | <Description>     *
*  2018/10/26|    1.0.   | Shuyuan.Yu | Create file       *
***********************************************************/
#pragma once

#include "iostream"
#include "immintrin.h"
#include "opencv2/opencv.hpp"
#include "math.h"
#include "../tools/MathTools.h"
#include "../camera/camera.h"

using namespace std;

#define debug
#define USE_ID

class Lamp {
public:
	cv::Point2f top;//上方点
	cv::Point2f down;//下方点
	cv::Point2f corner[4];//包围lamp矩形的四个角，顺序:[0]右下，[1]右上，[2]左上, [3]左下
	float angle;//倾斜角度，与平面直角坐标系角度保持一致
public:
	Lamp();
	Lamp(cv::Point2f t, cv::Point2f d, float a, cv::Point2f co[4]);
};

/*
 @函数:sortFittedLamp
 @功能:lamp的sort快排
 */
bool sortFittedLamp(pair<vector<Lamp>::iterator, float> a, pair<vector<Lamp>::iterator, float> b);

class Armor {
public:
	Lamp lampL;//左灯柱
	Lamp lampR;//右灯柱
	pair<Lamp, bool> lampLL = make_pair(Lamp(), false);//侧灯柱(未使用)
	pair<Lamp, bool> lampRR = make_pair(Lamp(), false);//侧灯柱(未使用)
	cv::Point2f center;//装甲中心
	unsigned int id;//装甲数字
	enum ArmorType {//类型
		BigArmor,
		SmallArmor,
		Unknow
	} type;
	cv::Mat rvec;//偏转角(PNP得出,未使用PnP)
	cv::Mat tvec;//平移矩阵(距离+角度计算得出)
	ushort distance;//距离
public:
	Armor();
	Armor(Lamp L, Lamp R);
	Armor(Lamp L, Lamp R, ArmorType T);
};

class ArmorDetector
{
public:
	ArmorDetector(int rows, int cols);
	~ArmorDetector();
public:
	enum Enemy {
		BLUE,
		RED
	};
public:
	/**
	* @函数: loadClassifier
	* @描述: 加载分类器
	* @输入: string file_name_small 小装甲分类器路径名称
			 string file_name_big 大装甲分类器路径名称
	* @返回:找到的灯柱数量
	*/
	void loadClassifier(string file_name_small, string file_name_big);
	
	/**
	* @函数: getArmor2
	* @描述: 查找装甲板
	* @输入: cv::Mat &src 原彩色图CV_8UC3
			 uchar bright_thr_value 亮度阈值(若为0只用颜色检测)
			 uchar color_thr_value 颜色值
			 Enemy enemy 敌方颜色类型
			 int useblur = 0 腐蚀膨胀内核大小
	* @输入输出:Armor &OutputArmor 输出装甲
	* @返回:bool 是否找到装甲
	*/
	bool getArmor2(cv::Mat & src, Armor &OutputArmor, uchar bright_thr_value, uchar color_thr_value, Enemy enemy, int useblur);

	/**
	* @函数: drawArmors
	* @描述: 画装甲板
	*/
	void drawArmors(cv::Mat & src, vector<Armor> armors);
	void drawArmors(cv::Mat & src, Armor a);
private:
	/**
	* @函数: findLamps
	* @描述: 图像中找灯柱
	* @输入: cv::Mat &src 原彩色图CV_8UC3
			 uchar bright_thr_value 亮度阈值(若为0只用颜色检测)
			 uchar color_thr_value 颜色值
			 Enemy enemy 敌方颜色类型
			 int useblur = 0 腐蚀膨胀内核大小
    * @输入输出：vector<Lamp> &lamps 检测到的灯柱
	* @返回:找到的灯柱数量
	*/
	int findLamps(cv::Mat &src, vector<Lamp> &lamps, uchar bright_thr_value, uchar color_thr_value, Enemy enemy, int useblur = 0);

	/**
	* @函数: fitArmor
	* @描述: 灯柱拟合装甲
	* @输入: cv::Mat &src 原彩色图CV_8UC3
			 vector<Lamp> lamps 找到的灯柱
	* @输入输出:vector<Armor> &armors 找到的装甲
	* @返回:找到的灯柱数量
	*/
	int fitArmor(Mat &src, vector<Lamp> lamps, vector<Armor> &armors);
	
	/**
	* @函数: getDistance
	* @描述: 利用灯柱高度计算距离
	* @输入输出:Armor & a 装甲
	*/
	void getDistance(Armor & a);

	/**
	* @函数: getID
	* @描述: 利用SVM检测数字
	* @输入输出:cv::Mat &src 原图
				Armor &a 装甲
	* @返回:检测的ID
	*/
	unsigned int getID(cv::Mat &src, Armor &a);

	/**
	* @函数: selectArmor
	* @描述: 从检测到的多个选择距离最近装甲
	* @输入: vector<Armor> armors 输入多个装甲
	  @输入输出:  Armor &a 输出装甲
	* @返回:检测的ID
	*/
	void selectArmor(vector<Armor> armors, Armor &armor);
	
private:
	vector<cv::Point3f> BigArmorSize;//装甲实际尺寸，用于PNP检测距离
	vector<cv::Point3f> SmallArmorSize;//装甲实际尺寸，用于PNP检测距离
	vector<cv::Point2f> PointInBigArmorPic;//透视变换大装甲图片尺寸
	vector<cv::Point2f> PointInSmallArmorPic;//透视变换小装甲图片尺寸

	cv::HOGDescriptor *hog_small,*hog_big;//大小装甲HOG特征
	cv::Ptr<cv::ml::SVM> IDclassifier_small, IDclassifier_big;//大小装甲SVM分类器
	bool EnableClassifier;//允许使用分类器的标志


	cv::Mat gray, color_thr ,bright_thr;//三种预处理图像(灰度、颜色、亮度)
};