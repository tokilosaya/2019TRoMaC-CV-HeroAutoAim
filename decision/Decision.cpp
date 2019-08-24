#include "Decision.h"



Decision::Decision(){
	tvec42.create(cv::Size(1,3) , CV_32FC1);
	tvec42.at<float>(0, 0) = 50;
	tvec42.at<float>(0, 1) = 110;
	tvec42.at<float>(0, 2) = -130;
	tvec17.create(cv::Size(1, 3), CV_32FC1);
	tvec17.at<float>(0, 0) = 0;
	tvec17.at<float>(0, 1) = 50;
	tvec17.at<float>(0, 2) = -130;

	mode = 42;
	heat42 = 200;
	heat17 = 200;
	speed17 = 20;

	std::cout << "init Decision success" << std::endl;
	std::cout << "tvec42:" << std::endl;
	std::cout << tvec42 << std::endl;
	std::cout << "tvec17:" << std::endl;
	std::cout << tvec17 << std::endl;
}


Decision::~Decision()
{
}

void Decision::setState(int m, int h42, int h17, int speed) {
	mode = m;
	heat42 = h42;
	heat17 = h17;
	speed17 = speed;
}

std::tuple<cv::Mat, int, bool> Decision::make(){
	
	//std::cout << "hea42:" << heat42;
	//std::cout << "  heat17:" << heat17;;
	//std::cout << std::endl;
	if (mode == 42) {
		return  std::make_tuple(tvec42, 15 ,true);
	}
	else if (mode == 17) {
		return  std::make_tuple(tvec17, (int)speed17 ,false);
	}
	else if (mode == 1) {
		if(heat42 > 100)
			return  std::make_tuple(tvec42, 15 ,true);
		else
			return  std::make_tuple(tvec17, (int)speed17 ,false);
	}
	else
		return std::make_tuple(tvec42, 15 ,true);

	return std::make_tuple(cv::Mat(),1,false);
}
