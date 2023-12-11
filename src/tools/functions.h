//
// Created by dayan3847 on 14/11/23.
//

#ifndef FUNCTIONS_D_H
#define FUNCTIONS_D_H

#include "opencv2/opencv.hpp"
#include "Config.h"

namespace dayan
{
	void makeMeasurement(
			cv::Mat& inputFrameLab,
			cv::Mat& mask,
			cv::Mat& mean,
			cv::Mat& iCov,
			float const& umDistVal,
			float const& umLuzVal,
			int const& contourPointMinCount,
			int const& error,
			Circle*& circle,
			cv::Mat& Z,
			int const& frame
	)
	{
		makeMeasurement(
				inputFrameLab,
				mask,
				mean,
				iCov,
				umDistVal,
				umLuzVal,
				contourPointMinCount,
				error,
				circle
		);
		if (nullptr == circle)
			return;
		auto config = dayan::Config::getInstance();
		// Centro en pixeles
		cv::Mat centerPx = (cv::Mat_<float>(3, 1)
				<<
				circle->h,
				circle->k,
				1
		);
		// Centro en metros
		cv::Mat centerMe = config->kInv * centerPx;
		centerMe = centerMe / centerMe.at<float>(2);
		// Radio en pixeles
		float radiusPx = circle->r;
		// Radio en metros
		float radiusMe = radiusPx * config->kInv.at<float>(0, 0);
		// TODO pruebas de diferentes formas de calcular el radio
//		// Radio en metros (test2)
//		cv::Mat p1Px = (cv::Mat_<float>(3, 1)
//			<<
//			circle->h + circle->r,
//			circle->k,
//			1
//		);
//		cv::Mat p1Me = kInv * p1Px;
//		float radiusMeTest2 = p1Me.at<float>(0) - centerMe.at<float>(0);
//		// Radio en metros (test3)
//		cv::Mat p2Px = (cv::Mat_<float>(3, 1)
//			<<
//			circle->h,
//			circle->k + circle->r,
//			1
//		);
//		cv::Mat p2Me = kInv * p2Px;
//		float radiusMeTest3 = p2Me.at<float>(1) - centerMe.at<float>(1);

		auto m = Z.rows;

		auto x = centerMe.at<float>(0);
		auto y = centerMe.at<float>(1);
		auto r = radiusMe;
		Z.at<float>(0) = x;
		Z.at<float>(1) = y;
		Z.at<float>(2) = r;

		if (m <= 3)
		{
			config->list_Z.push_back(Z.clone());
			return;
		}

		cv::Mat Z_1 = config->list_Z.empty() ? cv::Mat::zeros(m, 1, CV_32F) : config->list_Z.back();

		auto x_k_1 = Z_1.at<float>(0);  // x_k-1
		auto y_k_1 = Z_1.at<float>(1);  // y_k-1

		float dt = config->dTimes[frame];

		auto dx = 0 == dt ? 0 : (x - x_k_1) / dt;
		auto dy = 0 == dt ? 0 : (y - y_k_1) / dt;
		Z.at<float>(3) = dx;
		Z.at<float>(4) = dy;

		if (m <= 5)
		{
			config->list_Z.push_back(Z.clone());
			return;
		}

		auto r_k_1 = Z_1.at<float>(2);  // r_k-1

		auto dr = 0 == dt ? 0 : (r - r_k_1) / dt;
		Z.at<float>(5) = dr;

		if (m <= 6)
		{
			config->list_Z.push_back(Z.clone());
			return;
		}

		float dt_k_1 = 0 == frame ? 0 : config->dTimes[frame - 1];
		auto dt_avg = (dt + dt_k_1) / 2;

		auto dx_k_1 = Z_1.at<float>(3);  // dx_k-1
		auto dy_k_1 = Z_1.at<float>(4);  // dy_k-1

		auto ddx = 0 == dt ? 0 : (dx - dx_k_1) / dt_avg;
		auto ddy = 0 == dt ? 0 : (dy - dy_k_1) / dt_avg;
		Z.at<float>(6) = ddx;
		Z.at<float>(7) = ddy;

		if (m <= 8)
		{
			config->list_Z.push_back(Z.clone());
			return;
		}

		auto dr_k_1 = Z_1.at<float>(5);  // dr_k-1

		auto ddr = 0 == dt ? 0 : (dr - dr_k_1) / dt;
		Z.at<float>(8) = ddr;

		config->list_Z.push_back(Z.clone());
	}

	// pintar el circulo a partir de una medicion
	void drawCircleByZ(
			const cv::Mat& inputFrame,
			const cv::Mat& ZZ,
			const cv::Scalar& color
	)
	{
		auto config = dayan::Config::getInstance();

		// Centro en metros
		cv::Mat centerMe = (cv::Mat_<float>(3, 1)
				<<
				ZZ.at<float>(0),
				ZZ.at<float>(1),
				1
		);
		// Centro en pixeles
		cv::Mat centerPx = config->k * centerMe;
		centerPx = centerPx / centerPx.at<float>(2);
		cv::Point center(cvRound(centerPx.at<float>(0)), cvRound(centerPx.at<float>(1)));
		// Radio en metros
		float radiusMe = ZZ.at<float>(2);
		// Radio en pixeles
		float radiusPx = radiusMe * config->k.at<float>(0, 0);
		int radius = cvRound(radiusPx);
		if (radius > 0)
			cv::circle(inputFrame, center, radius, color,
					3, 8, 0);
	}

	// pintar el circulo a partir de un estado
	void drawCircleByX(
			const cv::Mat& inputFrame,
			const cv::Mat& XX,
			const cv::Scalar& color
	)
	{
		auto config = dayan::Config::getInstance();

		auto X = XX.at<float>(0);
		auto Y = XX.at<float>(1);
		auto Z = XX.at<float>(2);

		auto Rm = config->radio;

		cv::Mat ZZ = (cv::Mat_<float>(3, 1)
				<<
				X / Z, // x
				Y / Z, // y
				Rm / Z // r
		);

		drawCircleByZ(inputFrame, ZZ, color);
	}

} // dayan
#endif //FUNCTIONS_D_H
