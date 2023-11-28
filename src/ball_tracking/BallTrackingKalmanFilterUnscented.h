//
// Created by dayan3847 on 20/11/23.
//

#ifndef BALLTRACKINGKALMANFILTERUNSCENTED_H
#define BALLTRACKINGKALMANFILTERUNSCENTED_H

#include "opencv2/opencv.hpp"
#include "../tools/Config.h"
#include "../kf/KalmanFilterUnscented.h"

namespace dayan
{

	class BallTrackingKalmanFilterUnscented : public KalmanFilterUnscented
	{
	public:
		// Constructor
		BallTrackingKalmanFilterUnscented()
			: KalmanFilterUnscented(6, 5)
		{
			Q = 1e-6 * cv::Mat::eye(6, 6, CV_32F);
			R = 1 * cv::Mat::eye(5, 5, CV_32F);
			P = 1 * cv::Mat::eye(6, 6, CV_32F);
			I = cv::Mat::eye(6, 6, CV_32F);
		}
		// Inicializa el primer estado a partir de la primera medida
		void init_X() override
		{
			auto config = dayan::Config::getInstance();

			auto x = this->Z.at<float>(0);
			auto y = this->Z.at<float>(1);
			auto r = this->Z.at<float>(4);

			auto Z = config->radio / r;
			auto X = x * Z;
			auto Y = y * Z;

			this->X = (cv::Mat_<float>(6, 1)
				<<
				X,
				Y,
				Z,
				0,
				0,
				0
			);
		}
		// Update Matrix A
		void update_A(const int& dt) override
		{
			auto dts = (float)dt / 1000;
			A = (cv::Mat_<float>(6, 6)
				<<
				1, 0, 0, dts, 0, 0,
				0, 1, 0, 0, dts, 0,
				0, 0, 1, 0, 0, dts,
				0, 0, 0, 1, 0, 0,
				0, 0, 0, 0, 1, 0,
				0, 0, 0, 0, 0, 1
			);
		}

	protected:
		void get_h(const cv::Mat& XX, cv::Mat& h)
		{
			auto config = dayan::Config::getInstance();
			float X = XX.at<float>(0);
			float Y = XX.at<float>(1);
			float Z = XX.at<float>(2);
			float Xp = XX.at<float>(3);
			float Yp = XX.at<float>(4);
			float Zp = XX.at<float>(5);

			h = (cv::Mat_<float>(5, 1)
				<<
				X / Z,
				Y / Z,
				(Xp + (X / Z) * Zp) / Z,
				(Yp + (Y / Z) * Zp) / Z,
				config->radio / Z
			);
		}
	};

} // dayan

#endif //BALLTRACKINGKALMANFILTERUNSCENTED_H
