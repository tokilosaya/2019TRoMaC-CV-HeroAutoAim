#pragma once

#include <stdio.h>
#include <string.h>

#include <fcntl.h> 
#include <unistd.h>

#include <termios.h>

#include <sys/ioctl.h>
#include <errno.h>

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;

class Uart{
	public:
		Uart();
		~Uart();

		/**
		* @函数: Open
		* @描述: 打开串口
		 */
		bool Open(const char * device, int _speed, int _parity, bool _should_block);

		/**
		* @函数: restart
		* @描述: 串口重启
		 */
		bool restart();
		
		/**
		* @函数: ReadData
		* @描述: 读取串口
		 */
		bool ReadData(unsigned char data[23], unsigned char & mode, float & yaw, short & pitch, float & timestamp, short & heat42, short & heat17, unsigned char & bulletSpeed17);
		
		/**
		* @函数: SendData
		* @描述: 发送字节数据
		 */
		bool SendData(char* data,int len);

		/**
		* @函数: send
		* @描述: 发送编码数据
		 */
		bool send(float yaw, short pitch, float timestamp,bool shoot42 ,bool shoot17);

		/**
		* @函数: SendMiss();
		* @描述: 发送无识别对象指令
		 */
		bool SendMiss();
		/**
		* @函数: Close
		* @描述: 关闭串口
		 */
		void Close();
	private:
		const char *uart_path;
		int fd;
		int speed;
		int parity;
		bool should_block;		

};
