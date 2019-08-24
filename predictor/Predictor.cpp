#include "Predictor.h"

Predictor::Predictor(float pC){
	pitchCenter = pC;

	int stateSize = 4;
	int measureSize = 2;
	Eigen::MatrixXd A(stateSize, stateSize);
	A << 1, 0, 1, 0,
		0, 1, 0, 1,
		0, 0, 1, 0,
		0, 0, 0, 1 ;

	Eigen::MatrixXd H(measureSize, stateSize);
	H << 1, 0, 0, 0,
		0, 1, 0, 0;

	Eigen::MatrixXd P(stateSize, stateSize);
	P << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	Eigen::MatrixXd Q(stateSize, stateSize);
	Q << 5, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 5, 0,
		0, 0, 0, 1;

	Eigen::MatrixXd R(measureSize, measureSize);
	R << 200, 0,
		0, 200;

	KF.init(stateSize, measureSize, A, P, R, Q, H);

	x.resize(stateSize);
	x << 0, 0, 0, 0;
}

Predictor::~Predictor()
{
}

pair<cv::Point2f, bool> Predictor::predict(float yaw, float pitch, float yaw_now, float pitch_now,float distance, int bulletSpeed) {
	
	cv::Point2f predictPoint = kalmanPredict(yaw, pitch, distance/bulletSpeed * 10);
	predictPoint.y = compensaBallisticDrop(predictPoint.y, distance, bulletSpeed);
	if (abs(predictPoint.x - yaw_now) < 2 && abs(predictPoint.y - pitch_now) < 30) {
		//cout << "shoot" << endl;
		return make_pair(predictPoint, true);//可以打击
	}
	else {
		//cout << "cant shoot" << endl;
		return make_pair(predictPoint, false);//不能打击
	}
}

cv::Point2f Predictor::kalmanPredict(float yaw, float pitch, float time){
	if (abs(x(0) - yaw) > 200) {
		x(0) = (double)yaw;
		x(2) = 0;
	}
	if (abs(x(2) - pitch) > 200) {
		x(1) = (double)pitch;
		x(3) = 0;
	}
	Eigen::VectorXd z(2);
	Eigen::VectorXd output;

	z << (double)yaw, (double)pitch;
	KF.predict(x);
	KF.update(x, z);
	//cout << " X:" << x(0) << " v:" << x(2) << endl;
	double predictYaw = x(0) + (time + 12) * x(2); 
	double predictPitch = x(1) + (time + 12) * x(3);

	return cv::Point2f((float)predictYaw, (float)predictPitch);
}

float Predictor::compensaBallisticDrop(float input,float distance, int bulletSpeed) {
	double t = double(distance / (bulletSpeed * 1000));//飞行时间：水平距离/水平速度 = 距离*cos/速度*cos  = 距离/速度
	double dropDistance = 0.5 * 9.8 * pow(t, 2) * 1000;

	double angle = -(double)(input - pitchCenter) / 8192. * 360.;//计算敌方位置距pitch水平的角度

	double compensaedHeight = (double)distance * sin(angle / 180. * CV_PI) + dropDistance;
	double compensaedAngle = atan(compensaedHeight / (double)distance / cos(angle / 180. * CV_PI))*180. / CV_PI;

// 	cout << "bulletSpeed"<< bulletSpeed ;
// 	cout << "dropDistance" << dropDistance;
// 	cout << "compensaedHeight" <<compensaedHeight;
// 	cout << compensaedAngle << compensaedAngle;
// 	cout << endl;
	
	double pitch = -compensaedAngle * 8192. / 360. + pitchCenter;

	return (float)pitch;
}