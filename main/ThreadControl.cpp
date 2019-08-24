#include "ThreadControl.h"

string cameraMatrixFileName("/home/tromac/program/hero1907/camera/intrinsics.yml");
string smallArmorModelFileName("/home/tromac/program/hero1907/detector/classifier.xml");
string bigArmorModelFileName("/home/tromac/program/hero1907/detector/big_armor.xml");

#define picWidth 640
#define picHeight 640

atomic<bool> Exit(false);

Uart Serial;

bool InitAccomplish = false;//初始化成功标志
unsigned int needSynchronizedTime(3);//需要重新同步时间标志

atomic<bool> EnableSynchronizedTime(true);//允许同步时间标志
atomic<bool> InitUartAccomplish(false);//串口通信初始化成功标志

atomic<double> startTime(0);//开始计时的系统时间

CoordinatesFusion fusion(6342,6894,7289);
/*初始化预判*/
Predictor predictor(6894);

/*初始化决策*/
Decision decision;

MyCamera cam;

processer::processer(){
}

processer::~processer(){
}


void processer::InfoReadThreadLoop() {
	/*初始化串口*/
	Serial.Open("/dev/ttyUSB0",B115200,0,true);
	
	InitUartAccomplish = true;
	/*定义串口读回数据*/
	unsigned char data[23];//读取23个字节
	unsigned char mode = 0,bulletSpeed = 20;
	startTime = cv::getTickCount();
	float yaw = 0, timestamp = 0 , lastTimestamp = -1 ;
	short pitch = 0, startYaw = 0, heat42 = 0, heat17 = 0;
	/*读取循环*/
	while (true) {
		if (Serial.ReadData(data, mode, yaw, pitch, timestamp, heat42 , heat17, bulletSpeed)) {
			//cout << " mode" << (int)mode; 
			//cout << " yaw" << yaw;
			//cout << " pitch" << pitch;
			//cout << " time" << timestamp;
			//cout << " heatA" << heat42; 
			//cout << " heatB" << heat17;
			//cout << " bulletSpeed" << (int)bulletSpeed ;
			//cout << endl;

			if (abs(timestamp - (cv::getTickCount() - startTime) / cv::getTickFrequency()) > 0.010)
				needSynchronizedTime++;
			else
				needSynchronizedTime = 0;
			if (timestamp != 0){
				if (lastTimestamp < timestamp && needSynchronizedTime != 3) {
					fusion.pushGimblaPose(yaw, pitch, timestamp);//返回云台编码值
					decision.setState((int)mode, (int)heat42 , (int)heat17, (int)bulletSpeed);
					lastTimestamp == timestamp;
				}
				/*同步逻辑*/
				if (needSynchronizedTime >= 3 && EnableSynchronizedTime) {
					//同步主控时间轴，主控读取时间发回，做差求出开始的系统时间，串口通信加上解算时间约为1.3ms
					startTime = cv::getTickCount() - ((double)timestamp)*cv::getTickFrequency() + 0.0013;
					fusion.clear();
					needSynchronizedTime = 0;
					cout << "init TimeStamp success" << endl;
				}
			}
		}
		else {
		//	cout << "fales" << endl;
		}
		if (Exit)
			break;
	}
	
}

void processer::ArmorDetectorThreadLoop(){
	while (!InitUartAccomplish);//等待串口初始化
	/*初始化装甲识别*/
	ArmorDetector armorDetector(picHeight, picWidth);
	armorDetector.loadClassifier(smallArmorModelFileName, bigArmorModelFileName);
	/*初始化摄像头*/
	cam.open();
	cam.loadMatrix(cameraMatrixFileName);

	double lastFindTimestamp = 0;

	cv::Mat frame;//固定取图的地址
	double timestamp = 0;

	pair<Point2f,bool> predictValue;
	Point2f targetValue;

	while (true) {
		if (cam.getFrame(frame)) {
			timestamp = (cv::getTickCount() - startTime) / cv::getTickFrequency() - 0.004;//获取图片的时间戳
			Armor armor;
			if (armorDetector.getArmor2(frame, armor, 200, 100, ArmorDetector::RED, 0)) {
			//if (armorDetector.getArmor2(frame, armor, 200, 100, ArmorDetector::BLUE, 0)) {
				tuple <cv::Mat, int, bool> modeState = decision.make();

				if (fusion.caculateTargetPose(cam.pix2angle(armor.center, get<2>(modeState)), get<0>(modeState), (float)armor.distance, timestamp, targetValue)) {//这一步用于得到目标的编码坐标

					pair<float, short> gimblaState = fusion.getLatestData();
					predictValue = predictor.predict(targetValue.x, targetValue.y, gimblaState.first, gimblaState.second, (float)armor.distance, get<1>(modeState));//这一步用于预判和下坠补偿

					if (get<2>(modeState)){
						if(Serial.send(predictValue.first.x, (short)predictValue.first.y, (float)timestamp, predictValue.second, false))
							Serial.restart();
					}
					else{
						if(Serial.send(predictValue.first.x, (short)predictValue.first.y, (float)timestamp, false, predictValue.second))
							Serial.restart();
					}
				}
				lastFindTimestamp = timestamp;
				armorDetector.drawArmors(frame, armor);
			}
			else if (timestamp - lastFindTimestamp < 0.3 && timestamp - lastFindTimestamp > 0) {
				Serial.Send(targetValue.x, (short)targetValue.y, (float)timestamp, false, false);
			}
			else {
				if(!Serial.SendMiss()){
					Serial.restart();
				}
			}
			circle(frame, cam.picCenter(), 10, Scalar(0, 255, 0));
			cv::imshow("frame", frame);
			char key = waitKey(1);
			if (key == 27) {
				Exit = true;
				break;
			}
		}
	}
}