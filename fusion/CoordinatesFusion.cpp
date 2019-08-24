#include "CoordinatesFusion.h"
#ifdef PostProcesser_H

mutex mtxGimblaPose;

CoordinatesFusion::CoordinatesFusion(){
	
}

CoordinatesFusion::CoordinatesFusion(short PMIN, short PC, short PMAX){
	PoseDataSaveTime = 5;
	pitchMin = PMIN;
	pitchCenter = PC;
	pitchMax = PMIN;
}

CoordinatesFusion::~CoordinatesFusion(){
}

void CoordinatesFusion::pushGimblaPose(float yaw, short pitch, float timestamp){
	GimblaPose pose;
	pose.yaw = yaw;
	pose.pitch = pitch;
	pose.timestamp = (double)timestamp;
	LatestGimblaPoseTime = pose.timestamp;
	
	mtxGimblaPose.lock();
	historyGimblaPose.push_back(pose);
	if (!historyGimblaPose.empty()) {
		std::vector<GimblaPose>::iterator iter = historyGimblaPose.begin();
		while ((historyGimblaPose.back().timestamp - iter->timestamp) > 10 ||
			historyGimblaPose.back().timestamp < iter->timestamp) {//这里不能用等号，因为一旦把第一个元素也删了之后.empty下调用back无效
			historyGimblaPose.erase(iter);
		}
	}
	mtxGimblaPose.unlock();
}

bool CoordinatesFusion::caculateTargetPose(cv::Point2f angle, cv::Mat tvecDeviation,  float distance, double timestamp, cv::Point2f &targetPose) {
	GimblaPose gimblaPose;
	if (!interpolateRobotPose(timestamp, gimblaPose)) {
		//cout << "interpolateRobotPoe error" << endl;
		return false;
	}

	Mat tvec(1, 3, CV_32FC1);

	tvec.at<float>(0, 0) = (float)distance * tan(angle.x / 180 * CV_PI) + tvecDeviation.at<float>(0, 0);
	tvec.at<float>(0, 1) = (float)distance * tan(angle.y / 180 * CV_PI) + tvecDeviation.at<float>(0, 1);
	tvec.at<float>(0, 2) = (float)distance + tvecDeviation.at<float>(0, 2);

	Point2f correctedAngle;
	correctedAngle.x = atan2(tvec.at<float>(0, 0), tvec.at<float>(0, 2)) * 180 / CV_PI;
	correctedAngle.y = atan2(tvec.at<float>(0, 1), tvec.at<float>(0, 2)) * 180 / CV_PI;

	targetPose.x = gimblaPose.yaw - correctedAngle.x ;

	targetPose.y = gimblaPose.pitch - short(correctedAngle.y * 8192. / 360.);
	if (targetPose.y > pitchMax)
		targetPose.y = pitchMax;
	else if(targetPose.y < pitchMin)
		targetPose.y = pitchMin;
	//std::cout << "pitch" << gimblaPose.pitch << endl;// << " x " << angle.y * 8192 / 360.*1.3<< "  " << gimblaPose.pitch - 6940<< std::endl;
	return true;
}

pair<float, short> CoordinatesFusion::getLatestData(){
	GimblaPose pose;
	mtxGimblaPose.lock();
	if (!historyGimblaPose.empty()) {
		pose = historyGimblaPose.back();
		mtxGimblaPose.unlock();
	}
	else {
		mtxGimblaPose.unlock();
		return make_pair(0., 0.);
	}
	return make_pair(pose.yaw, pose.pitch);
}



bool CoordinatesFusion::interpolateRobotPose(double timestamp , GimblaPose &pose){
	if (timestamp == 0 || historyGimblaPose.size() <= 2) {
		std::cout << "error historyGimblaPose.size()" << historyGimblaPose.size() << "timestamp" << timestamp<<std::endl;
		return false;
	}
	//cout << "timestamp" << timestamp <<"Gimbla" <<historyGimblaPoseData.back().timestamp << endl;
	//时间段内无法找到插值
	if (LatestGimblaPoseTime + 0.080 < timestamp ||
		LatestGimblaPoseTime - 0.50 > timestamp) {
		std::cout << "interploateError: latest GimblaPoseTime has a big difference from now " << timestamp << " " << LatestGimblaPoseTime << std::endl;
		return false;
	}
	/*插值时间大于主控返回时间时的前向线性预测值*/
	if (timestamp > LatestGimblaPoseTime) {
		GimblaPose prePose, afterPose;
		mtxGimblaPose.lock();
		prePose = (*(historyGimblaPose.end() - 2));
		afterPose = (*(historyGimblaPose.end() - 1));
		mtxGimblaPose.unlock();
		if (afterPose.timestamp - prePose.timestamp > 0.030 ||
			afterPose.timestamp - prePose.timestamp <= 0) {
			std::cout << "Interpolate error:neighbor timestamp too long" << afterPose.timestamp << " " << prePose.timestamp << std::endl;
			return false;
		}
		else {
			float rate = ((float)timestamp - afterPose.timestamp) / (afterPose.timestamp - prePose.timestamp);
			pose.timestamp = timestamp;
			pose.yaw = (afterPose.yaw - prePose.yaw) * rate + afterPose.yaw;
			pose.pitch = (short)((float)(afterPose.pitch - prePose.pitch) * rate + (float)afterPose.pitch);
			//cout << "front" << " rate"<<rate << "time1" << timestamp << "pre"<< prePose.timestamp <<"after"<< afterPose.timestamp <<" imu"<< pose.imu << " pitch"<<pose.pitch << endl;
			return true;
		}
	}
	/*插值时间处于主控返回时间之内的线性插值*/
	else {
		mtxGimblaPose.lock();
		GimblaPose prePose, afterPose;
		std::vector<GimblaPose>::iterator r = historyGimblaPose.end() - 1;
		while (timestamp <= r->timestamp) {
			if (r != historyGimblaPose.begin())
				--r;
			else {
				mtxGimblaPose.unlock();
				std::cout << "mpu value interpolate mistake:find touch startPoint" << std::endl;
				return false;
			}
		}
		prePose = *r;
		afterPose = *(r + 1);
		mtxGimblaPose.unlock();
		if (afterPose.timestamp - prePose.timestamp > 0.05) {
			std::cout << "mpu value interpolate mistake:neighbor timestamp too long" << std::endl;
			return false;
		}
		float rate = ((float)timestamp - prePose.timestamp) / (afterPose.timestamp - prePose.timestamp);
		pose.timestamp = timestamp;
		pose.yaw = (afterPose.yaw - prePose.yaw) * rate + prePose.yaw;
		pose.pitch = (short)((float)(afterPose.pitch - prePose.pitch) * rate + (float)prePose.pitch);
		return true;
	}
}

void CoordinatesFusion::clear(){
	historyGimblaPose.clear();
}
#endif