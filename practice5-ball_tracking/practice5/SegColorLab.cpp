#include <iostream>
#include <chrono>
#include <thread>
#include "opencv2/opencv.hpp"
#include "lib/arturo/functions.h"
#include "lib/arturo/Circle/Circle.h"
#include "lib/tools.h"


int main(int argc, char** argv)
{
	std::string filename = argc < 2 ? "./media/ball_green_video.mkv" : argv[1];
	std::string colorModel = argc < 3 ? "./media/ball_green_model.png" : argv[2];
	std::string timesFilename = argc < 4 ? "./media/ball_green_times.txt" : argv[3];

	// Read delta times
	std::vector<int> deltaTimes;
	dayan::readDeltaTimes(timesFilename, deltaTimes);

	// Input
	cv::VideoCapture inputVideoCapture;

	dayan::getVideoCapture(filename, inputVideoCapture);

	cv::Mat colorFrame = cv::imread(colorModel);

	std::string inputWinName = "Input";
	std::string maskWinName = "Mask";
	cv::namedWindow(inputWinName, 1);
	cv::namedWindow(maskWinName, 1);

	int sleep = 1;
	cv::createTrackbar("Sleep", inputWinName, &sleep, 1);
	// Slide 1 (distancia)
	int dSlidePos = 200;
//	arturo::barData umDist(40. / SLIDE_MAX, 47);
//	cv::createTrackbar("umDist", inputWinName, &dSlidePos, SLIDE_MAX, arturo::umDistChange,
//		(void*)&umDist);
//	arturo::umDistChange(SLIDE_MAX, (void*)&umDist);
	// Slide 2 (luz)
	int lSlidePos = 16;
	arturo::barData umLuz(100. / SLIDE_MAX, 0);
	cv::createTrackbar("umLuz", inputWinName, &lSlidePos, SLIDE_MAX, arturo::umLuzChange,
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
	// Crear un vector vacio de Z (mediciones)
	std::vector<cv::Mat> list_Z;

	// KalmanFilter
	cv::KalmanFilter kalmanFilter(6, 5, 0);
	// R
	setIdentity(kalmanFilter.processNoiseCov, Scalar::all(1e-4));
	// Q
	setIdentity(kalmanFilter.measurementNoiseCov, Scalar::all(1));
	// P
	setIdentity(kalmanFilter.errorCovPost, Scalar::all(.1));

	int frameCount = -1;
	do
	{
		std::cout << "\033[1;32m" << "Frame: " << ++frameCount << "\033[0m" << std::endl;
		// We capture an image, and validate that the operation worked.
		inputVideoCapture >> inputFrame;
		if (inputFrame.empty())
			break;

		dt += deltaTimes[frameCount];

		std::cout << "dt: " << dt << std::endl;

		// Matrix A
		kalmanFilter.transitionMatrix =
			(Mat_<float>(6, 6)
				<<
				1, 0, 0, dt, 0, 0,
				0, 1, 0, 0, dt, 0,
				0, 0, 1, 0, 0, dt,
				0, 0, 0, 1, 0, 0,
				0, 0, 0, 0, 1, 0,
				0, 0, 0, 0, 0, 1
			);

//		// Matrix H
//		kalmanFilter.measurementMatrix =



		// In the first iteration we initialize the images that we will use to store results.
		if (0 == frameCount)
		{
			arturo::convertLab(colorFrame, inputFrameLab);

			cv::Mat mMask = cv::Mat::ones(colorFrame.size(), CV_8UC1);
			arturo::MeaniCov(inputFrameLab, mMask, mean, iCov);

			cv::Size sz(inputFrame.cols, inputFrame.rows);
			mask = cv::Mat::ones(sz, CV_8U);
		}
		else
		{
			arturo::convertLab(inputFrame, inputFrameLab);
		}
		Mat prediction = kalmanFilter.predict();

		Circle* c = nullptr;
		cv::Mat Z;
//		std::cout << "\033[1;32m" << "umDistVal: " << umDist.val << "\033[0m" << std::endl;
//		std::cout << "\033[1;32m" << "umLuz.val: " << umLuz.val << "\033[0m" << std::endl;
		dayan::makeMeasurement(
			inputFrameLab,
			mask,
			mean,
			iCov,
			50,  //umDist.val,
			umLuz.val,
			contourPointMinCount,
			error,
			c,
			list_Z,
			Z,
			dt
		);
		if (nullptr == c)
		{
			std::cout << "\033[1;31m" << "No se encontro el circulo" << "\033[0m" << std::endl;
			continue;
		}
		dayan::printMat(Z, "Z");
		//dayan::drawCircle(inputFrame, c);
		dayan::drawCircleByZ(inputFrame, Z, cv::Scalar(255, 0, 0));

		if (0 == frameCount)
		{
			dayan::getX(Z, kalmanFilter.statePre);
			kalmanFilter.statePost = kalmanFilter.statePre;
			dayan::printMat(kalmanFilter.statePre, "X");
		}
		else
		{
			cv::Mat h;
			dayan::getH(kalmanFilter.statePre, h, kalmanFilter.measurementMatrix);

//			cv::Mat corrected = kalmanFilter.correct(Z);
			Mat HxQxHt = kalmanFilter.measurementMatrix * kalmanFilter.errorCovPre * kalmanFilter.measurementMatrix.t()
				+ kalmanFilter.measurementNoiseCov;
			Mat HxQxHt_inv;
			cv::invert(HxQxHt, HxQxHt_inv, cv::DECOMP_LU);
			kalmanFilter.gain = kalmanFilter.errorCovPre * kalmanFilter.measurementMatrix.t() * HxQxHt_inv;
			kalmanFilter.statePost = kalmanFilter.statePre + kalmanFilter.gain * (Z - h);
			kalmanFilter.errorCovPost =
				(cv::Mat::eye(6, 6, CV_32F) - kalmanFilter.gain * kalmanFilter.measurementMatrix)
					* kalmanFilter.errorCovPre;

			dayan::printMat(kalmanFilter.statePost, "corrected");
//			 corrected color green
			dayan::drawCircleByX(inputFrame, kalmanFilter.statePost, cv::Scalar(0, 255, 0));
//			dayan::drawCircleByX(inputFrame, corrected, cv::Scalar(0, 255, 0));
		}
		cv::Mat predicted = kalmanFilter.predict();
//		kalmanFilter.statePre = kalmanFilter.transitionMatrix * kalmanFilter.statePost;
//		kalmanFilter.errorCovPre =
//			kalmanFilter.transitionMatrix * kalmanFilter.errorCovPost * kalmanFilter.transitionMatrix.t()
//				+ kalmanFilter.processNoiseCov;
//
//		dayan::printMat(kalmanFilter.statePre, "predicted");
		// predicted color red
//		dayan::drawCircleByX(inputFrame, kalmanFilter.statePre, cv::Scalar(0, 0, 255));
		dayan::drawCircleByX(inputFrame, predicted, cv::Scalar(0, 0, 255));

		// Show images
		imshow(inputWinName, inputFrame);
		imshow(maskWinName, mask);

		std::cout << "error permitido: " << error << std::endl;

		if (0 < sleep)
			std::this_thread::sleep_for(std::chrono::seconds(1));

		dt = 0;
		// If the user presses a key, the cycle ends.
//		break;
	} while (cv::waitKeyEx(30) < 0);

	imwrite("./media/LastFrame.png", inputFrame);

	// Close windows that were opened.
	cv::destroyWindow(inputWinName);
	cv::destroyWindow(maskWinName);

	std::cout << "\033[1;32m" << "End" << "\033[0m" << std::endl;
	return 0;
}
