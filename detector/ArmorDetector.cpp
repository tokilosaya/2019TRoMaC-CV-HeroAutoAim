#include "ArmorDetector.h"

extern MyCamera cam;
int sendtime = 0;


Lamp::Lamp(){
}

Lamp::Lamp(cv::Point2f t, cv::Point2f d, float a , cv::Point2f co[4]) {
	top = t;
	down = d;
	angle = a;
	for(int i = 0; i < 4;++i)
		corner[i] = co[i];
}

Armor::Armor() {
	distance = 0;
	id = -1;
}

Armor::Armor(Lamp L, Lamp R) {
	lampL = L;
	lampR = R;
	center = (L.top + L.down + R.top + R.down)/4;
	distance = 0;
	id = -1;
}

Armor::Armor(Lamp L, Lamp R, ArmorType T) {
	lampL = L;
	lampR = R;
	center = (L.top + L.down + R.top + R.down) / 4;
	type = T;
	distance = 0;
	id = -1;
}

ArmorDetector::ArmorDetector(int rows, int cols) {
	EnableClassifier = false;

	SmallArmorSize.push_back(cv::Point3f(-61.5, -26.75, 0.0));
	SmallArmorSize.push_back(cv::Point3f(61.5, -26.75, 0.0));
	SmallArmorSize.push_back(cv::Point3f(61.5, 26.75, 0.0));
	SmallArmorSize.push_back(cv::Point3f(-61.5, 26.75, 0.0));

	BigArmorSize.push_back(cv::Point3f(-110.0, -28.5, 0.0));
	BigArmorSize.push_back(cv::Point3f(110.0, -28.5, 0.0));
	BigArmorSize.push_back(cv::Point3f(110.0, 28.5, 0.0));
	BigArmorSize.push_back(cv::Point3f(-110.0, 28.5, 0.0));

	PointInBigArmorPic.push_back(cv::Point2f(0, 48));
	PointInBigArmorPic.push_back(cv::Point2f(128, 48));
	PointInBigArmorPic.push_back(cv::Point2f(128, 16));
	PointInBigArmorPic.push_back(cv::Point2f(0, 16));

	PointInSmallArmorPic.push_back(cv::Point2f(0, 48));
	PointInSmallArmorPic.push_back(cv::Point2f(64, 48));
	PointInSmallArmorPic.push_back(cv::Point2f(64, 16));
	PointInSmallArmorPic.push_back(cv::Point2f(0, 16));

	color_thr.create(rows, cols, CV_8UC1);
}

ArmorDetector::~ArmorDetector(){
}

void ArmorDetector::loadClassifier(string file_name_small,string file_name_big){
	IDclassifier_small =  cv::ml::SVM::load(file_name_small);
	hog_small = new cv::HOGDescriptor(cv::Size(64, 64), cv::Size(16, 16), cv::Size(8, 8), cv::Size(8, 8), 9);
	IDclassifier_big = cv::ml::SVM::load(file_name_big);
	hog_big = new cv::HOGDescriptor(cv::Size(128, 64), cv::Size(16, 16), cv::Size(8, 8), cv::Size(8, 8), 9);
	if (!IDclassifier_small->empty() && !IDclassifier_big->empty()) {
		EnableClassifier = true;
		cout << "Read Classifier accomplish" << endl;
	}
	else
		cout << "Read Classifier failed" << endl;
}

void ArmorDetector::drawArmors(cv::Mat & src, vector<Armor> armors) {
	for (Armor a:armors) {
		cv::line(src, a.lampL.corner[2], a.lampR.corner[3], cv::Scalar(0, 255, 0));
		cv::line(src, a.lampL.corner[1], a.lampR.corner[0], cv::Scalar(0, 255, 255));
		cv::putText(src, to_string(a.id),a.lampL.top,1,1,cv::Scalar(255,255,255));
		cv::putText(src, to_string(a.rvec.at<double>(0, 0) / CV_PI * 180),a.lampR.top - cv::Point2f(0, 30),1,1,cv::Scalar(255,255,255));
		cv::putText(src, to_string(a.rvec.at<double>(0, 1) / CV_PI * 180),a.lampR.top - cv::Point2f(0, 15),1,1,cv::Scalar(255,255,255));
		cv::putText(src, to_string(a.rvec.at<double>(0, 2) / CV_PI * 180),a.lampR.top - cv::Point2f(0, 0),1,1,cv::Scalar(255,255,255));
		cv::putText(src, to_string(a.tvec.at<double>(0, 0)), a.lampR.down + cv::Point2f(0, 15), 1, 1, cv::Scalar(255, 255, 255));
		cv::putText(src, to_string(a.tvec.at<double>(0, 1)), a.lampR.down + cv::Point2f(0, 30), 1, 1, cv::Scalar(255, 255, 255));
		cv::putText(src, to_string(a.tvec.at<double>(0, 2)), a.lampR.down + cv::Point2f(0, 45), 1, 1, cv::Scalar(255, 255, 255));
		//cv::putText(src, "0", a.lampL.down, 1, 1, cv::Scalar(255, 255, 255));
		//cv::putText(src, "1", a.lampR.down, 1, 1, cv::Scalar(255, 255, 255));
		if (a.lampLL.second) {
			cv::line(src, a.lampL.top, a.lampLL.first.down, cv::Scalar(0, 0, 255));
			cv::line(src, a.lampL.down, a.lampLL.first.top, cv::Scalar(0, 0, 255));
		}
		if (a.lampRR.second) {
			cv::line(src, a.lampR.top, a.lampRR.first.down, cv::Scalar(0, 0, 255));
			cv::line(src, a.lampR.down, a.lampRR.first.top, cv::Scalar(0, 0, 255));
		}
	}
}

void ArmorDetector::drawArmors(cv::Mat & src, Armor a) {
	cv::line(src, a.lampL.corner[2], a.lampR.corner[3], cv::Scalar(0, 255, 0));
	cv::line(src, a.lampL.corner[1], a.lampR.corner[0], cv::Scalar(0, 255, 255));
	cv::putText(src, to_string(a.id), a.lampL.top, 1, 1, cv::Scalar(255, 255, 255));
	cv::putText(src, to_string(a.distance), a.lampR.top - cv::Point2f(0, 30), 1, 1, cv::Scalar(255, 255, 255));
	//cv::putText(src, "0", a.lampL.down, 1, 1, cv::Scalar(255, 255, 255));
	//cv::putText(src, "1", a.lampR.down, 1, 1, cv::Scalar(255, 255, 255));
	if (a.lampLL.second) {
		cv::line(src, a.lampL.top, a.lampLL.first.down, cv::Scalar(0, 0, 255));
		cv::line(src, a.lampL.down, a.lampLL.first.top, cv::Scalar(0, 0, 255));
	}
	if (a.lampRR.second) {
		cv::line(src, a.lampR.top, a.lampRR.first.down, cv::Scalar(0, 0, 255));
		cv::line(src, a.lampR.down, a.lampRR.first.top, cv::Scalar(0, 0, 255));
	}
}

bool sortFittedLamp(pair<vector<Lamp>::iterator, float> a, pair<vector<Lamp>::iterator, float> b) {
	return a.second < b.second;
}

int ArmorDetector::findLamps(cv::Mat & src, vector<Lamp> &possible_lamps, uchar bright_thr_value, uchar color_thr_value, Enemy enemy, int useblur){
	if (bright_thr_value != 0) {
		cv::cvtColor(src, gray, CV_BGR2GRAY);
		cv::threshold(gray, bright_thr, bright_thr_value, 255, cv::THRESH_BINARY);
	}
	int total_pixel = src.rows * src.cols;
	const uchar * ptr_src = src.data;
	uchar *ptr_dst = color_thr.data;
	const uchar *ptr_dst_end = color_thr.data + total_pixel;
	uchar b, r;
	if (enemy == BLUE)
		for (; ptr_dst != ptr_dst_end; ptr_src += 1, ptr_dst += 1) {
			b = *ptr_src;
			ptr_src += 2;
			r = *ptr_src;
			if (b - r > color_thr_value)
				*ptr_dst = 255;
			else
				*ptr_dst = 0;
		}
	else if (enemy == RED)
		for (; ptr_dst != ptr_dst_end; ptr_src += 1, ptr_dst += 1) {
			b = *ptr_src;
			ptr_src += 2;
			r = *ptr_src;
			if (r - b > color_thr_value)
				*ptr_dst = 255;
			else
				*ptr_dst = 0;
		}
	if (useblur != 0)
		dilate(color_thr, color_thr, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(useblur, useblur)));

#ifdef debug
	imshow("color_thr", color_thr);
	if (bright_thr_value != 0)
		imshow("bright_thr", bright_thr);
#endif
	vector< cv::Vec4i > hierarchy;
	vector< vector <cv::Point> > bright_contours, light_contours, possible_lampContours;

	if (bright_thr_value != 0)
		cv::findContours(bright_thr, bright_contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	cv::findContours(color_thr, light_contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	if (bright_thr_value != 0) {
		for (unsigned int i = 0; i < bright_contours.size(); ++i) {
			if (bright_contours[i].size() < 10)
				continue;
			for (unsigned int j = 0; j < light_contours.size(); ++j) {
				if (cv::pointPolygonTest(light_contours[j], bright_contours[i][0], false) >= 0.0) {
					possible_lampContours.push_back(bright_contours[i]);
				}
			}
		}
	}
	else {
		for (unsigned int i = 0; i < light_contours.size(); ++i) {
			possible_lampContours.push_back(light_contours[i]);
		}
	}
	for (vector< vector <cv::Point> >::iterator c = possible_lampContours.begin(); c != possible_lampContours.end(); ++c) {
		cv::RotatedRect rr;
		rr = cv::minAreaRect(*c);
		cv::Point2f vertex[4];
		rr.points(vertex);
		if (rr.size.width > rr.size.height) {
			if (rr.size.width < rr.size.height * 1.5)
				continue;
			cv::Point2f co[4];
			co[0] = vertex[1];
			co[1] = vertex[0];
			co[2] = vertex[3];
			co[3] = vertex[2];
			possible_lamps.emplace_back((vertex[2] + vertex[3]) / 2,
				(vertex[0] + vertex[1]) / 2,
				-rr.angle,
				co);
		}
		else {
			if (rr.size.width*1.5 > rr.size.height)
				continue;
			cv::Point2f co[4];
			co[0] = vertex[0];
			co[1] = vertex[3];
			co[2] = vertex[2];
			co[3] = vertex[1];
			possible_lamps.emplace_back((vertex[1] + vertex[2]) / 2,
				(vertex[0] + vertex[3]) / 2,
				-rr.angle + 90,
				co);
		}
	}
	return possible_lamps.size();
}

int ArmorDetector::fitArmor(Mat &src,vector<Lamp> lamps, vector<Armor> &armors){
	vector<Lamp> _lamps;
	for (Lamp l : lamps)
		_lamps.push_back(l);
	for (vector<Lamp>::iterator L1 = _lamps.begin(); L1 <= _lamps.end() - 1; ++L1) {
		vector<pair<vector<Lamp>::iterator, float> > fittedLamps;
		//对L1进行灯柱拟合，记录灯柱拟合的值返回到Lamps中
		for (vector<Lamp>::iterator L2 = L1 + 1; L2 != _lamps.end(); ++L2) {
			if (abs(L1->angle - L2->angle) > 7) {
				continue;
			}
			double angle = cal_angle(L1->top - L2->top, L1->down - L2->down);
			if (abs(angle) > 10) {
				continue;
			}
			angle = cal_angle(L1->top - L2->top, L1->top - L1->down);
			if (abs(angle) > 115 || abs(angle) < 65) {
				continue;
			}
			float D1 = dist2D(L1->top, L1->down);
			float D2 = dist2D(L2->top, L2->down);
			if (abs(D1 - D2) > D1*1.5) {
				continue;
			}
			float distance = dist2D((L1->top + L1->down) / 2, (L2->top + L2->down) / 2);
			fittedLamps.emplace_back(L2, distance);
		}
		if (fittedLamps.size() == 0) {
			continue;
		}
		sort(fittedLamps.begin(), fittedLamps.end(), sortFittedLamp);
		float length = dist2D(L1->top, L1->down);

		for (int i = 0; i < fittedLamps.size(); ++i) {
			Armor pArmor;
			float gapRate = fittedLamps[i].second / length;
			if (gapRate > 1.2 && gapRate < 3.0) {
				if (L1->down.x + L1->top.x < fittedLamps[i].first->down.x + fittedLamps[i].first->top.x)
					pArmor = Armor(*L1, *fittedLamps[i].first, Armor::SmallArmor);
				else
					pArmor = Armor(*fittedLamps[i].first, *L1, Armor::SmallArmor);
			}
			else if (gapRate > 3.0 && gapRate < 5.) {
				if (L1->down.x + L1->top.x < fittedLamps[i].first->down.x + fittedLamps[i].first->top.x)
					pArmor = Armor(*L1, *fittedLamps[i].first, Armor::BigArmor);
				else
					pArmor = Armor(*fittedLamps[i].first, *L1, Armor::BigArmor);
			}
			else
				continue;

			if (getID(src, pArmor) != 0) {
				armors.push_back(pArmor);
				_lamps.erase(fittedLamps[i].first);
				_lamps.erase(L1);
				--L1;
				break;
			}
		}
	}
	return armors.size();
}

void ArmorDetector::getDistance(Armor & a) {
	cv::Point2f topL = a.lampL.top;
	cv::Point2f topR = a.lampR.top;
	cv::Point2f downL = a.lampL.down;
	cv::Point2f downR = a.lampR.down;

	float height = dist2D(cam.undistortPoints(topL), cam.undistortPoints(downL))*0.5 + dist2D(cam.undistortPoints(topR), cam.undistortPoints(downR))*0.5;
	ushort pridectDistance;
	//利用灯柱高度计算距离
	pridectDistance = (ushort)(65000. / height);
	a.distance = pridectDistance;
}

unsigned int ArmorDetector::getID(cv::Mat & src, Armor & a){
#ifndef USE_ID
	a.id = 2;
	return a.id;
#else
	if (a.type == Armor::SmallArmor) {
		double startIDtime = cv::getTickCount();
		cv::Mat pic(64, 64, CV_8UC3);
		vector<cv::Point2f> points;
		points.push_back(a.lampL.down);
		points.push_back(a.lampR.down);
		points.push_back(a.lampR.top);
		points.push_back(a.lampL.top);
		cv::Mat trans = cv::getPerspectiveTransform(points, PointInSmallArmorPic);
		cv::warpPerspective(src, pic, trans, pic.size());
		cv::cvtColor(pic, pic, cv::COLOR_BGR2GRAY);
		cv::rectangle(pic,cv::Point(0,0),cv::Point(9,63),cv::Scalar(0),-1);
		cv::rectangle(pic,cv::Point(54,0),cv::Point(63,63),cv::Scalar(0),-1);
		cv::threshold(pic,pic, 20, 255,CV_THRESH_OTSU);
		//cout << "size1" << pic.size() << endl;
#ifdef debug
		imshow("trans", pic);
		
#endif
		/*SVM detector*/
		if (EnableClassifier) {
			vector<float> descriptors;//存放结果         
			hog_small->compute(pic, descriptors, cv::Size(1, 1), cv::Size(0, 0)); //Hog特征计算        
			cv::Mat  SVMtrainMat = cv::Mat(1, descriptors.size(), CV_32FC1);
			int n = 0;
			for (vector<float>::iterator iter = descriptors.begin(); iter != descriptors.end(); iter++) {
				SVMtrainMat.at<float>(0, n) = *iter;//第i个样本的特征向量中的第n个元素  
				n++;
			}
			SVMtrainMat.convertTo(SVMtrainMat, CV_32FC1);
			a.id = (unsigned int)IDclassifier_small->predict(SVMtrainMat);
		}
		if(a.id == 8)
			a.id =0;
		else if(a.id == 0)
			a.id = 8;
		return a.id;
	}
	else if (a.type == Armor::BigArmor) {
		cv::Mat _pic(64, 128, CV_8UC3);
		vector<cv::Point2f> points;
		points.push_back(a.lampL.down);
		points.push_back(a.lampR.down);
		points.push_back(a.lampR.top);
		points.push_back(a.lampL.top);
		cv::Mat trans = cv::getPerspectiveTransform(points, PointInBigArmorPic);
		cv::warpPerspective(src, _pic, trans, _pic.size());
		cv::cvtColor(_pic, _pic, cv::COLOR_BGR2GRAY);
		cv::Mat tem = _pic(cv::Rect(cv::Point(31,0),cv::Point(95,64)));
 		cv::Mat pic;
		tem.copyTo(pic);
		cv::threshold(pic,pic, 20, 255,CV_THRESH_OTSU);
		//cout << "size2" << pic.size() << endl;
		
		/*KNN detector*/
		if (EnableClassifier) {
			vector<float> descriptors;//存放结果         
			hog_small->compute(pic, descriptors, cv::Size(1, 1), cv::Size(0, 0)); //Hog特征计算        
			cv::Mat  SVMtrainMat = cv::Mat(1, descriptors.size(), CV_32FC1);
			int n = 0;
			for (vector<float>::iterator iter = descriptors.begin(); iter != descriptors.end(); iter++) {
				SVMtrainMat.at<float>(0, n) = *iter;//第i个样本的特征向量中的第n个元素  
				n++;
			}
			SVMtrainMat.convertTo(SVMtrainMat, CV_32FC1);
			a.id = (unsigned int)IDclassifier_small->predict(SVMtrainMat);
		}
		if(a.id == 8)
			a.id =0;
		else if(a.id == 0)
			a.id = 8;
		return a.id;
	}
#endif
}

void ArmorDetector::selectArmor(vector<Armor> armors, Armor &armor){
	short minDistance = 10000;
	for (Armor &a : armors) {
		if (a.distance == 0)
			continue;
		if (a.distance < minDistance) {
			armor = a;
			minDistance = a.distance;
		}
	}

}

bool ArmorDetector::getArmor2(cv::Mat & src, Armor& OutputArmors , uchar bright_thr_value, uchar color_thr_value, Enemy enemy, int useblur){
	vector<Lamp> lamps;
	vector<Armor> armors;
	//Armor selectedArmor;
	findLamps(src, lamps, bright_thr_value, color_thr_value, enemy, useblur);
	if (lamps.size() < 2)
		return false;
	fitArmor(src, lamps, armors);
	for (Armor &a : armors)
		getDistance(a);
	if (armors.size() == 0)
		return false;
	selectArmor(armors, OutputArmors);
	return true;
}