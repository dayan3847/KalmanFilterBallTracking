#include <iostream>
#include <chrono>
#include <thread>
#include "opencv2/opencv.hpp"
#include "lib/arturo/functions.h"
#include "lib/arturo/Circle/Circle.h"
#include "lib/dayan/tools.h"
//#include "lib/dayan/Config.h"
#include "lib/dayan/KalmanFilterExtended.h"


int main(int argc, char** argv)
{
	std::string data_path = argc < 2 ? "ball_orange" : argv[1];
//	dayan::Config::loadData(data_path);

	std::string file_color = "./media/" + data_path + "/color.png";
	std::string file_diameter_cm = "./media/" + data_path + "/diameter_cm.txt";
	std::string file_times = "./media/" + data_path + "/times.txt";
	std::string file_video = "./media/" + data_path + "/video.mkv";
	std::string file_k = "./media/" + data_path + "/k.yaml";

	// Read color frame
	cv::Mat colorFrame = cv::imread(file_color);
	// Read diameter
	float diameter_cm;
	std::ifstream file_diameter_cm_stream(file_diameter_cm);
	file_diameter_cm_stream >> diameter_cm;
	// Read delta times
	std::vector<int> deltaTimes;
	dayan::readDeltaTimes(file_times, deltaTimes);
	// Input
	cv::VideoCapture inputVideoCapture;
	dayan::getVideoCapture(file_video, inputVideoCapture);
	// K (camera matrix)
	cv::Mat k;
	dayan::readMatFromFile(k, file_k);
	cv::Mat kInv;
	cv::invert(k, kInv);

	dayan::printMat(k, "k");

	// Radio cm
	float radio_cm = diameter_cm / 2;
	// Radio Real (en metros)
	float radioReal = radio_cm / 100;

	std::string inputWinName = "Input";
	std::string maskWinName = "Mask";
	cv::namedWindow(inputWinName, 1);
	cv::namedWindow(maskWinName, 1);

	int sleep = 0;
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

//	// KalmanFilter
//	cv::KalmanFilter kalmanFilter0(6, 5, 0);
//	setIdentity(kalmanFilter0.processNoiseCov, Scalar::all(1e-4));
//	setIdentity(kalmanFilter0.measurementNoiseCov, Scalar::all(10));
//	setIdentity(kalmanFilter0.errorCovPost, Scalar::all(.1));
//
//	dayan::printMat(kalmanFilter0.processNoiseCov, "processNoiseCov");
//	dayan::printMat(kalmanFilter0.measurementNoiseCov, "measurementNoiseCov");
//	dayan::printMat(kalmanFilter0.errorCovPost, "errorCovPost");

	// KalmanFilter
	dayan::KalmanFilterExtended kalmanFilter;
//	setIdentity(kalmanFilter.Q, Scalar::all(1e-4));
	kalmanFilter.Q = (Mat_<float>(6, 6)
		<<
		1e-4, 0, 0, 0, 0, 0,
		0, 1e-4, 0, 0, 0, 0,
		0, 0, 1e-4, 0, 0, 0,
		0, 0, 0, 1e-4, 0, 0,
		0, 0, 0, 0, 1e-4, 0,
		0, 0, 0, 0, 0, 1e-4
	);
//	setIdentity(kalmanFilter.R, Scalar::all(10));
	kalmanFilter.R = (Mat_<float>(5, 5)
		<<
		10, 0, 0, 0, 0,
		0, 10, 0, 0, 0,
		0, 0, 10, 0, 0,
		0, 0, 0, 10, 0,
		0, 0, 0, 0, 10
	);
//	setIdentity(kalmanFilter.P, Scalar::all(.1));
	kalmanFilter.P = (Mat_<float>(6, 6)
		<<
		.1, 0, 0, 0, 0, 0,
		0, .1, 0, 0, 0, 0,
		0, 0, .1, 0, 0, 0,
		0, 0, 0, .1, 0, 0,
		0, 0, 0, 0, .1, 0,
		0, 0, 0, 0, 0, .1
	);
	dayan::printMat(kalmanFilter.Q, "Q");
	dayan::printMat(kalmanFilter.R, "R");
	dayan::printMat(kalmanFilter.P, "P");

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
		kalmanFilter.A = (Mat_<float>(6, 6)
			<<
			1, 0, 0, dt, 0, 0,
			0, 1, 0, 0, dt, 0,
			0, 0, 1, 0, 0, dt,
			0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 1
		);

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
//		kalmanFilter.predict();

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
		//dayan::printMat(Z, "Z");
		//dayan::drawCircle(inputFrame, c);
		dayan::drawCircleByZ(inputFrame, Z, cv::Scalar(255, 0, 0));

		if (0 == frameCount)
		{
			dayan::getX(Z, kalmanFilter.X);
//			kalmanFilter.statePost = kalmanFilter.statePre;
//			dayan::printMat(kalmanFilter.statePre, "X");
		}
		else
		{
			kalmanFilter.predict_correct(Z);
//			dayan::printMat(kalmanFilter.X, "corrected");
//			dayan::printMat(kalmanFilter.Xp, "predicted");
			// predicted color red
			//dayan::drawCircleByX(inputFrame, kalmanFilter.Xp, cv::Scalar(0, 0, 255));
			// corrected color green
			dayan::drawCircleByX(inputFrame, kalmanFilter.X, cv::Scalar(0, 255, 0));
		}


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
