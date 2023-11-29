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
			Q = 1e-6 * cv::Mat::eye(n, n, CV_32F);
			R = 1 * cv::Mat::eye(m, m, CV_32F);
			P = 1 * cv::Mat::eye(n, n, CV_32F);
		}

		// Inicializa el primer estado a partir de la primera medida
		void init_X() override
		{
			auto config = dayan::Config::getInstance();

			auto x = this->Z.at<float>(0);
			auto y = this->Z.at<float>(1);
			auto r = this->Z.at<float>(2);

			auto Z = config->radio / r;
			auto X = x * Z;
			auto Y = y * Z;

			this->X = (cv::Mat_<float>(n, 1)
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
		void update_A(const float& dt) override
		{
			A = (cv::Mat_<float>(n, n)
					<<
					1, 0, 0, dt, 0, 0,
					0, 1, 0, 0, dt, 0,
					0, 0, 1, 0, 0, dt,
					0, 0, 0, 1, 0, 0,
					0, 0, 0, 0, 1, 0,
					0, 0, 0, 0, 0, 1
			);
		}

	protected:
		void get_h(const cv::Mat& XX, cv::Mat& h)
		{
			auto config = dayan::Config::getInstance();

			auto X = XX.at<float>(0);
			auto Y = XX.at<float>(1);
			auto Z = XX.at<float>(2);
			auto dX = XX.at<float>(3);
			auto dY = XX.at<float>(4);
			auto dZ = XX.at<float>(5);

			auto Rm = config->radio;

			auto Z2 = Z * Z;

			h = (cv::Mat_<float>(5, 1)
					<<
					X / Z, // x
					Y / Z, // y
					Rm / Z, // r
					-X * dZ / Z2 + dX / Z, // dx
					-Y * dZ / Z2 + dY / Z // dy
			);
		}
	};

} // dayan

#endif //BALLTRACKINGKALMANFILTERUNSCENTED_H
