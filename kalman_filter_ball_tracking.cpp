#include <iostream>
#include <chrono>
#include <thread>
#include "opencv2/opencv.hpp"
#include "lib/arturoemx/functions.h"
#include "lib/arturoemx/Circle/Circle.h"
#include "src/tools/tools.h"
#include "src/tools/Config.h"
#include "src/tools/functions.h"
#include "src/kf/KalmanFilterType.h"
#include "src/ball_tracking/BallTrackingKalmanFilterExtended.h"
#include "src/ball_tracking/BallTrackingKalmanFilterExtendedImplicit.h"

int main(int argc, char** argv)
{
	std::string data_path = argc < 2 ? "ball_tennis" : argv[1];
	dayan::KalmanFilterType kfType = argc < 3
		                                 ? dayan::KalmanFilterType::Extended
		                                 : (dayan::KalmanFilterType)atoi(argv[2]);
	auto config = dayan::Config::getInstance(data_path);

	std::string inputWinName = "Input";
	std::string maskWinName = "Mask";

	// KalmanFilter
	dayan::KalmanFilter* kalmanFilter = nullptr;
	switch (kfType)
	{
	case dayan::KalmanFilterType::Extended:
		kalmanFilter = new dayan::BallTrackingKalmanFilterExtended();
		inputWinName = "Extended Kalman Filter";
		break;
	case dayan::KalmanFilterType::ExtendedImplicit:
		kalmanFilter = new dayan::BallTrackingKalmanFilterExtendedImplicit();
		inputWinName = "Implicit Extended Kalman Filter";
		break;
	default:
		std::cout << "\033[1;31m" << "KalmanFilterType not supported" << "\033[0m" << std::endl;
		return -1;
	}
	cv::namedWindow(inputWinName, 1);
	cv::namedWindow(maskWinName, 1);

	int sleep = 0;
	cv::createTrackbar("Sleep", inputWinName, &sleep, 1);
	// Slide 1 (distancia)
	//	int dSlidePos = 200;
	//	arturo::barData umDist(40. / SLIDE_MAX, 47);
	//	cv::createTrackbar("umDist", inputWinName, &dSlidePos, SLIDE_MAX, arturo::umDistChange,
	//		(void*)&umDist);
	//	arturo::umDistChange(SLIDE_MAX, (void*)&umDist);
	// Slide 2 (luz)
	int lSlidePos = 16;
	arturo::barData umLuz(100. / SLIDE_MAX, 0);
	cv::createTrackbar("umLuz",
		inputWinName,
		&lSlidePos,
		SLIDE_MAX,
		arturo::umLuzChange,
		(void*)&umLuz);
	arturo::umLuzChange(0, (void*)&umLuz);
	// Slide 3 (error permitido)
	int error = 5;
	cv::createTrackbar("Error", inputWinName, &error, 100);
	// Slide 3 (error permitido)
	int contourPointMinCount = 50;
	cv::createTrackbar("Contour Point Count Min", inputWinName, &contourPointMinCount, 100);

	cv::Mat inputFrame, inputFrameLab, mask, mean, iCov;

	int dt = 0;
	int frameCount = -1;
	do
	{
		std::cout << "\033[1;32m" << "Frame: " << ++frameCount << "\033[0m" << std::endl;
		// We capture an image, and validate that the operation worked.
		config->video >> inputFrame;
		if (inputFrame.empty())
			break;

		dt += config->dTimes[frameCount];
		std::cout << "dt: " << dt << std::endl;

		// In the first iteration we initialize the images that we will use to store results.
		if (0 == frameCount)
		{
			arturo::convertLab(config->color, inputFrameLab);

			cv::Mat mMask = cv::Mat::ones(config->color.size(), CV_8UC1);
			arturo::MeaniCov(inputFrameLab, mMask, mean, iCov);

			cv::Size sz(inputFrame.cols, inputFrame.rows);
			mask = cv::Mat::ones(sz, CV_8U);
		}
		else
		{
			arturo::convertLab(inputFrame, inputFrameLab);
		}
		Circle* c = nullptr;
		//		std::cout << "\033[1;32m" << "umDistVal: " << umDist.val << "\033[0m" << std::endl;
		//		std::cout << "\033[1;32m" << "umLuz.val: " << umLuz.val << "\033[0m" << std::endl;
		dayan::makeMeasurement(
			inputFrameLab,
			mask,
			mean,
			iCov,
			50,
			//umDist.val,
			umLuz.val,
			contourPointMinCount,
			error,
			c,
			kalmanFilter->Z,
			dt
		);
		if (nullptr == c)
		{
			std::cout << "\033[1;31m" << "No se encontro el circulo" << "\033[0m" << std::endl;
			continue;
		}
		//dayan::printMat(Z, "Z");
		//dayan::drawCircle(inputFrame, c);
		dayan::drawCircleByZ(inputFrame, kalmanFilter->Z, cv::Scalar(255, 0, 0));

		if (0 == frameCount)
		{
			imwrite("./data/" + data_path + "/FirstFrame.png", inputFrame);
			kalmanFilter->init_X();
		}
		else
		{
			kalmanFilter->predict_correct(dt);
			//			dayan::printMat(kalmanFilter->X, "corrected");
			//			dayan::printMat(kalmanFilter->Xp, "predicted");
			// predicted color red
			//dayan::drawCircleByX(inputFrame, kalmanFilter->Xp, cv::Scalar(0, 0, 255));
			// corrected color green
			dayan::drawCircleByX(inputFrame, kalmanFilter->X, cv::Scalar(0, 255, 0));
		}

		// Show images
		imshow(maskWinName, mask);
		imshow(inputWinName, inputFrame);

		//std::cout << "error permitido: " << error << std::endl;

		if (0 < sleep)
			std::this_thread::sleep_for(std::chrono::seconds(1));

		dt = 0;
		// If the user presses a key, the cycle ends.
		//		break;
	}
	while (cv::waitKeyEx(30) < 0);

	imwrite("./data/" + data_path + "/LastFrame.png", inputFrame);

	// Close windows that were opened.
	cv::destroyWindow(maskWinName);
	cv::destroyWindow(inputWinName);

	std::cout << "\033[1;32m" << "End" << "\033[0m" << std::endl;
	return 0;
}
