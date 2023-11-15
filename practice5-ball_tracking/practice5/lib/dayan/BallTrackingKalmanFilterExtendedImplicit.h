//
// Created by dayan3847 on 14/11/23.
//

#ifndef BALLTRACKINGKALMANFILTEREXTENDEDIMPLICIT_H
#define BALLTRACKINGKALMANFILTEREXTENDEDIMPLICIT_H

#include "opencv2/opencv.hpp"
#include "Config.h"
#include "KalmanFilterExtendedImplicit.h"

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
		void update_h_H() override
		{
			auto config = dayan::Config::getInstance();

			float X = this->Xp.at<float>(0);
			float Y = this->Xp.at<float>(1);
			float Z = this->Xp.at<float>(2);
			float Xp = this->Xp.at<float>(3);
			float Yp = this->Xp.at<float>(4);
			float Zp = this->Xp.at<float>(5);

			h = (cv::Mat_<float>(5, 1)
				<<
				X / Z,
				Y / Z,
				(Xp + (X / Z) * Zp) / Z,
				(Yp + (Y / Z) * Zp) / Z,
				config->radio / Z
			);
			auto Z2 = Z * Z;
			auto Z3 = Z2 * Z;
			auto Zp_Z2 = Zp / Z2;
			H = (cv::Mat_<float>(5, 6)
				<<
				1 / Z, 0, -X / Z2, 0, 0, 0,
				0, 1 / Z, -Y / Z2, 0, 0, 0,
				Zp_Z2, 0, Xp / Z2 - 2 * (X * Zp + Z * Xp) / Z3, 1 / Z, 0, X / Z2,
				0, Zp_Z2, Yp / Z2 - 2 * (Y * Zp + Z * Yp) / Z3, 0, 1 / Z, Y / Z2,
				0, 0, -config->radio / Z2, 0, 0, 0
			);
		}
	};

} // dayan

#endif //BALLTRACKINGKALMANFILTEREXTENDEDIMPLICIT_H
