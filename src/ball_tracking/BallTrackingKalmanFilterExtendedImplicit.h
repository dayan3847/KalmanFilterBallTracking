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
		// Constructor
		BallTrackingKalmanFilterExtendedImplicit()
		{
			Q = (cv::Mat_<float>(6, 6)
				<<
				1e-4, 0, 0, 0, 0, 0,
				0, 1e-4, 0, 0, 0, 0,
				0, 0, 1e-4, 0, 0, 0,
				0, 0, 0, 1e-4, 0, 0,
				0, 0, 0, 0, 1e-4, 0,
				0, 0, 0, 0, 0, 1e-4
			);
			R = (cv::Mat_<float>(5, 5)
				<<
				10, 0, 0, 0, 0,
				0, 10, 0, 0, 0,
				0, 0, 10, 0, 0,
				0, 0, 0, 10, 0,
				0, 0, 0, 0, 10
			);
			P = (cv::Mat_<float>(6, 6)
				<<
				.1, 0, 0, 0, 0, 0,
				0, .1, 0, 0, 0, 0,
				0, 0, .1, 0, 0, 0,
				0, 0, 0, .1, 0, 0,
				0, 0, 0, 0, .1, 0,
				0, 0, 0, 0, 0, .1
			);
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

			float X = this->Xp.at<float>(0);
			float Y = this->Xp.at<float>(1);
			float Z = this->Xp.at<float>(2);
			float Xp = this->Xp.at<float>(3);
			float Yp = this->Xp.at<float>(4);
			float Zp = this->Xp.at<float>(5);

			auto x = this->Z.at<float>(0);
			auto y = this->Z.at<float>(1);
			auto xp = this->Z.at<float>(2);
			auto yp = this->Z.at<float>(3);
			auto r = this->Z.at<float>(4);

			h = (cv::Mat_<float>(5, 1)
				<<
				x - X / Z,
				y - Y / Z,
				xp - (Xp + x * Zp) / Z,
				yp - (Yp + y * Zp) / Z,
				r - config->radio / Z
			);
			auto Z2 = Z * Z;
			auto _1_Z = -1 / Z;
			H = (cv::Mat_<float>(5, 6)
				<<
				_1_Z, 0, X / Z2, 0, 0, 0,
				0, _1_Z, Y / Z2, 0, 0, 0,
				0, 0, (Xp + x * Zp) / Z2, _1_Z, 0, -x / Z,
				0, 0, (Yp + y * Zp) / Z2, 0, _1_Z, -y / Z,
				0, 0, config->radio / Z2, 0, 0, 0
			);
			auto Zp_Z = -Zp / Z;
			J = (cv::Mat_<float>(5, 5)
				<<
				1, 0, 0, 0, 0,
				0, 1, 0, 0, 0,
				Zp_Z, 0, 1, 0, 0,
				0, Zp_Z, 0, 1, 0,
				0, 0, 0, 0, 1
			);
		}
	};

} // dayan

#endif //BALLTRACKINGKALMANFILTEREXTENDEDIMPLICIT_H
