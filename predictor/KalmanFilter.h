#pragma once

#include "opencv2/opencv.hpp"
#include "Eigen/Dense"

namespace EigenKalman{
	class KalmanFilter
	{
	private:
		int stateSize;     //状态方程维数
		int measSize;      //测量参数个数
		Eigen::MatrixXd A; //状态转移矩阵
		Eigen::MatrixXd P; //协方差矩阵
		Eigen::MatrixXd H; //测量矩阵
		Eigen::MatrixXd R; //测量误差
		Eigen::MatrixXd Q; //预测误差
	public:
		KalmanFilter();
		~KalmanFilter(){}
		/**
		* @函数: init
		* @描述: 初始化kalman预判参数
		 */
		void init(int state, int means, Eigen::MatrixXd& _A, Eigen::MatrixXd& _P, Eigen::MatrixXd& _R, Eigen::MatrixXd& _Q, Eigen::MatrixXd& _H);
		/**
		* @函数: predict
		* @描述: kalman预测
		* @输入输出:Eigen::VectorXd &x 状态向量
		*/
		void predict(Eigen::VectorXd &x);
		/**
		* @函数: update
		* @描述: kalman更新
		* @输入: Eigen::VectorXd z_meas 测量参数向量
		* @输入输出:Eigen::VectorXd &x 状态向量
		 */
		void update(Eigen::VectorXd &x, Eigen::VectorXd z_meas);
	};
}