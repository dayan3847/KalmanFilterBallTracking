//
// Created by dayan3847 on 29/11/23.
//

#ifndef KALMANFILTERBALLTRACKING_BALLTRACKINGKALMANFILTEREXTENDED_6X6_H
#define KALMANFILTERBALLTRACKING_BALLTRACKINGKALMANFILTEREXTENDED_6X6_H

#include "opencv2/opencv.hpp"
#include "../tools/Config.h"
#include "../kf/KalmanFilter.h"
#include "../kf/KalmanFilterExtended.h"

namespace dayan
{

	class BallTrackingKalmanFilterExtended_6x6 : public KalmanFilterExtended
	{


	public:
		// Constructor
		BallTrackingKalmanFilterExtended_6x6()
				: KalmanFilterExtended(6, 6)
		{
			Q = 1e-4 * cv::Mat::eye(n, n, CV_32F);
			R = 1e-2 * cv::Mat::eye(m, m, CV_32F);
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
		void update_h_jacobians() override
		{
			auto config = dayan::Config::getInstance();

			auto Rm = config->radio;

			auto X = this->Xp.at<float>(0);
			auto Y = this->Xp.at<float>(1);
			auto Z = this->Xp.at<float>(2);
			auto dX = this->Xp.at<float>(3);
			auto dY = this->Xp.at<float>(4);
			auto dZ = this->Xp.at<float>(5);

			auto Z2 = Z * Z;
			auto Z3 = Z2 * Z;

			h = (cv::Mat_<float>(m, 1)
					<<
					X / Z, // x
					Y / Z, // y
					Rm / Z, // r
					-X * dZ / Z2 + dX / Z, // dx
					-Y * dZ / Z2 + dY / Z, // dy
					-Rm * dZ / Z2 // dr
			);

			H = (cv::Mat_<float>(m, n)
					<<
					1 / Z, 0, -X / Z2, 0, 0, 0,
					0, 1 / Z, -Y / Z2, 0, 0, 0,
					0, 0, -Rm / Z2, 0, 0, 0,
					-dZ / Z2, 0, 2 * X * dZ / Z3 - dX / Z2, 1 / Z, 0, -X / Z2,
					0, -dZ / Z2, 2 * Y * dZ / Z3 - dY / Z2, 0, 1 / Z, -Y / Z2,
					0, 0, 2 * Rm * dZ / Z3, 0, 0, -Rm / Z2
			);
		}
	};

} // dayan

#endif //KALMANFILTERBALLTRACKING_BALLTRACKINGKALMANFILTEREXTENDED_6X6_H
