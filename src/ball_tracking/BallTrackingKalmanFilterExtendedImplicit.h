//
// Created by dayan3847 on 14/11/23.
//

#ifndef BALLTRACKINGKALMANFILTEREXTENDEDIMPLICIT_H
#define BALLTRACKINGKALMANFILTEREXTENDEDIMPLICIT_H

#include "opencv2/opencv.hpp"
#include "../tools/Config.h"
#include "../kf/KalmanFilterExtendedImplicit.h"

namespace dayan
{

	class BallTrackingKalmanFilterExtendedImplicit : public KalmanFilterExtendedImplicit
	{
	public:
		BallTrackingKalmanFilterExtendedImplicit()
			: KalmanFilterExtendedImplicit(6, 5)
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
			A = (cv::Mat_<float>(6, 6)
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

			auto x = this->Z.at<float>(0);
			auto y = this->Z.at<float>(1);
			auto r = this->Z.at<float>(2);
			auto dx = this->Z.at<float>(3);
			auto dy = this->Z.at<float>(4);

			auto Rm = config->radio;

			auto Z2 = Z * Z;
			auto dZ_Z = dZ / Z;

			h = (cv::Mat_<float>(m, 1)
				<<
				-X / Z + x,
				-Y / Z + y,
				-Rm / Z + r,
				dx - dX / Z + dZ * x / Z,
				dy - dY / Z + dZ * y / Z
			);

			H = (cv::Mat_<float>(m, n)
				<<
				-1 / Z, 0, X / Z2, 0, 0, 0,
				0, -1 / Z, Y / Z2, 0, 0, 0,
				0, 0, Rm / Z2, 0, 0, 0,
				0, 0, dX / Z2 - dZ * x / Z2, -1 / Z, 0, x / Z,
				0, 0, dY / Z2 - dZ * y / Z2, 0, -1 / Z, y / Z
			);
			J = (cv::Mat_<float>(m, m)
				<<
				1, 0, 0, 0, 0,
				0, 1, 0, 0, 0,
				0, 0, 1, 0, 0,
				dZ_Z, 0, 0, 1, 0,
				0, dZ_Z, 0, 0, 1
			);
		}
	};

} // dayan

#endif //BALLTRACKINGKALMANFILTEREXTENDEDIMPLICIT_H
