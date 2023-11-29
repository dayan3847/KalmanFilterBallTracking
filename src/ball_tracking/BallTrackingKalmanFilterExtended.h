//
// Created by dayan3847 on 14/11/23.
//

#ifndef BALLTRACKINGKALMANFILTEREXTENDED_H
#define BALLTRACKINGKALMANFILTEREXTENDED_H

#include "opencv2/opencv.hpp"
#include "../tools/Config.h"
#include "../kf/KalmanFilter.h"
#include "../kf/KalmanFilterExtended.h"

namespace dayan
{

	class BallTrackingKalmanFilterExtended : public KalmanFilterExtended
	{
	public:
		// Constructor
		BallTrackingKalmanFilterExtended()
			: KalmanFilterExtended(6, 5)
		{
			Q = 1e-4 * cv::Mat::eye(n, n, CV_32F);
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
		void update_A(const int& dt) override
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
		void update_h_jacobians() override
		{
			auto config = dayan::Config::getInstance();

			auto X = this->Xp.at<float>(0);
			auto Y = this->Xp.at<float>(1);
			auto Z = this->Xp.at<float>(2);
			auto dX = this->Xp.at<float>(3);
			auto dY = this->Xp.at<float>(4);
			auto dZ = this->Xp.at<float>(5);

			auto Rm = config->radio;

			auto Z2 = Z * Z;
			auto Z3 = Z2 * Z;
			auto _1_Z = 1 / Z;
			auto _dZ_Z2 = -dZ / Z2;
			auto _2dZ_Z3 = 2 * dZ / Z3;

//			h_x = sp.Matrix([
//			[X / Z],  # x
//			[Y / Z],  # y
//			[Rm / Z],  # r
//			[-X * dZ / Z ** 2 + dX / Z],  # dx
//			[-Y * dZ / Z ** 2 + dY / Z],  # dy
//			])

			h = (cv::Mat_<float>(5, 1)
				<<
				X / Z, // x
				Y / Z, // y
				Rm / Z, // r
				-X * dZ / Z2 + dX / Z, // dx
				-Y * dZ / Z2 + dY / Z // dy
			);

			H = (cv::Mat_<float>(5, 6)
				<<
				_1_Z, 0, -X / Z2, 0, 0, 0,
				0, _1_Z, -Y / Z2, 0, 0, 0,
				0, 0, -Rm / Z2, 0, 0, 0,
				_dZ_Z2, 0, X * _2dZ_Z3 - dX / Z2, _1_Z, 0, -X / Z2,
				0, _dZ_Z2, Y * _2dZ_Z3 - dY / Z2, 0, _1_Z, -Y / Z2
			);
		}
	};

} // dayan

#endif //BALLTRACKINGKALMANFILTEREXTENDED_H
