#include "camera.h"
#include "math.h"
MyCamera::MyCamera() {
}

MyCamera::~MyCamera() {
}

bool MyCamera::open() {
	cam.open(0);
	if (cam.isOpened())
		return true;
	else
		return false;
}

bool MyCamera::getFrame(cv::Mat &img) {
	cam >> img;
	return true;
}

void MyCamera::restart() {
	/*重启摄像机程序*/
}
bool MyCamera::loadMatrix(string filename) {
	cv::FileStorage file(filename, cv::FileStorage::READ);
	if (file.isOpened()) {
		file["Matrix"] >> cameraMatrix;
		file["distCoeffs"] >> distCoeffs;
		loadedMatrix = true;
		file.release();
		cout << "read CameraMatrix accomplish" << endl;
		return true;
	}
	else {
		file.release();
		cout << "openFile error" << endl;
		return false;
	}
}
cv::Point2f MyCamera::picCenter()
{
	return cv::Point2f(cameraMatrix.at<double>(0, 2), cameraMatrix.at<double>(1, 2));
}

cv::Point2f MyCamera::pix2angle(cv::Point2f point, bool shoot42) {
	cv::Point2f angle;
	if (loadedMatrix) {
		angle.x = atan2(point.x - cameraMatrix.at<double>(0, 2), cameraMatrix.at<double>(0, 0)) * 180. / CV_PI;
		angle.y = atan2(cameraMatrix.at<double>(1, 2) - point.y, cameraMatrix.at<double>(1, 1)) * 180. / CV_PI;
	}
	else {
		angle.x = 0;
		angle.y = 0;
	}
	/*弹道不平行枪管轴的角度纠正*/
	if (shoot42) {
		angle.x += 0.0;
		angle.y += 0.0;
	}
	else {
		angle.x += 0.0;
		angle.y += 0.0;
	}
	//cout << "angle" << angle << endl;
	return angle;
}

void MyCamera::pix2angle(cv::Point2f point, cv::Point2f & angle, bool shoot42) {
	angle = pix2angle(point, shoot42);
}

cv::Point2f MyCamera::undistortPoints(cv::Point2f inputPoint) {
	if (!loadedMatrix) {
		return inputPoint;
	}
	double fx = cameraMatrix.at<double>(0, 0);
	double fy = cameraMatrix.at<double>(1, 1);
	double ux = cameraMatrix.at<double>(0, 2);
	double uy = cameraMatrix.at<double>(1, 2);

	double k1 = distCoeffs.at<double>(0, 0);
	double k2 = distCoeffs.at<double>(0, 1);
	double p1 = distCoeffs.at<double>(0, 2);
	double p2 = distCoeffs.at<double>(0, 3);
	double k3 = distCoeffs.at<double>(0, 4);

	cv::Point2d p = cv::Point2d((double)inputPoint.x, (double)inputPoint.y);

	//首先进行坐标转换；
	double xDistortion = (p.x - ux) / fx;
	double yDistortion = (p.y - uy) / fy;

	double xCorrected, yCorrected;

	double x0 = xDistortion;
	double y0 = yDistortion;

	//这里使用迭代的方式进行求解
	for (int j = 0; j < 10; j++) {
		double r2 = xDistortion * xDistortion + yDistortion * yDistortion;

		double distRadialA = 1 / (1. + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

		double deltaX = 2. * p1 * xDistortion * yDistortion + p2 * (r2 + 2. * xDistortion * xDistortion);
		double deltaY = p1 * (r2 + 2. * yDistortion * yDistortion) + 2. * p2 * xDistortion * yDistortion;

		xCorrected = (x0 - deltaX)* distRadialA;
		yCorrected = (y0 - deltaY)* distRadialA;

		xDistortion = xCorrected;
		yDistortion = yCorrected;
	}

	xCorrected = xCorrected * fx + ux;
	yCorrected = yCorrected * fy + uy;

	return cv::Point2f((float)xCorrected, (float)yCorrected);
}

void MyCamera::undistortPoints(cv::Point2f inputPoint, cv::Point2f & outputPoint) {
	outputPoint = undistortPoints(inputPoint);
}