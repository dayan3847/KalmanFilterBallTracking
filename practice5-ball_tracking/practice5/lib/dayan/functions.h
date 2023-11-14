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
		int const& dt
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


		cv::Mat prevZ = config->list_Z.empty() ? cv::Mat::zeros(5, 1, CV_32F) : config->list_Z.back();
		float xp = 0 == dt ? 0 : (centerMe.at<float>(0) - prevZ.at<float>(0)) / (float)dt;
		float yp = 0 == dt ? 0 : (centerMe.at<float>(1) - prevZ.at<float>(1)) / (float)dt;

		Z = (cv::Mat_<float>(5, 1)
			<<
			centerMe.at<float>(0),
			centerMe.at<float>(1),
			xp,
			yp,
			radiusMe
		);
		config->list_Z.push_back(Z);
	}

	// obtener un estado inicial a partir de una medicion
	void getX(const cv::Mat& ZZ, cv::Mat& XX)
	{
		auto config = dayan::Config::getInstance();

		auto x = ZZ.at<float>(0);
		auto y = ZZ.at<float>(1);
		auto r = ZZ.at<float>(4);

		auto Z = config->radio / r;
		auto X = x * Z;
		auto Y = y * Z;

		XX = (cv::Mat_<float>(6, 1)
			<<
			X,
			Y,
			Z,
			0,
			0,
			0
		);
	}
	// obtener medicion a partir del estado
	void getZ(const cv::Mat& X, const cv::Mat& H, cv::Mat& Z)
	{
		Z = H * X;
	}
	// a partir de un estado, obtener h(X) y su jacobiano
	void getH(const cv::Mat& XX, cv::Mat& h, cv::Mat& H)
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
		float radiusMe = ZZ.at<float>(4);
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

		float X = XX.at<float>(0);
		float Y = XX.at<float>(1);
		float Z = XX.at<float>(2);

		cv::Mat ZZ = (cv::Mat_<float>(5, 1)
			<<
			X / Z,
			Y / Z,
			0,
			0,
			config->radio / Z
		);

		drawCircleByZ(inputFrame, ZZ, color);
	}

} // dayan
#endif //FUNCTIONS_D_H
