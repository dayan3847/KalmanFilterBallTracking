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
	int dSlidePos = 211;
	arturo::barData umDist(40. / SLIDE_MAX, 10);
	cv::createTrackbar("umDist", inputWinName, &dSlidePos, SLIDE_MAX, arturo::umDistChange,
		(void*)&umDist);
	arturo::umDistChange(SLIDE_MAX, (void*)&umDist);
	// Slide 2 (luz)
	int lSlidePos = 16;
	arturo::barData umLuz(100. / SLIDE_MAX, 0);
	cv::createTrackbar("umLuz", inputWinName, &lSlidePos, SLIDE_MAX, arturo::umLuzChange,
		(void*)&umLuz);
	arturo::umLuzChange(0, (void*)&umLuz);
	// Slide 3 (error permitido)
	int error = 1;
	cv::createTrackbar("Error", inputWinName, &error, 100);
	// Slide 3 (error permitido)
	int contourPointMinCount = 50;
	cv::createTrackbar("Contour Point Count Min", inputWinName, &contourPointMinCount, 100);

	cv::Mat inputFrame, inputFrameLab, mask, mean, iCov;
	int dt = 0;
	int frameCount = -1;
	do
	{
		std::cout << "\033[1;32m" << "Frame: " << frameCount++ << "\033[0m" << std::endl;
		// We capture an image, and validate that the operation worked.
		inputVideoCapture >> inputFrame;
		if (inputFrame.empty())
			break;

		dt += deltaTimes[frameCount];

		std::cout << "dt: " << dt << std::endl;

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

		Circle* c = nullptr;
		dayan::makeMeasurement(
			inputFrameLab,
			mask,
			mean,
			iCov,
			umDist.val,
			umLuz.val,
			contourPointMinCount,
			error,
			c
		);
		if (nullptr == c)
		{
			std::cout << "No se encontro el circulo" << std::endl;
			continue;
		}

		cv::Point center(cvRound(c->h), cvRound(c->k));
		int radius = cvRound(c->r);
		cv::circle(inputFrame, center, radius, cv::Scalar(0, 0, 255),
			3, 8, 0);

		// Show images
		imshow(inputWinName, inputFrame);
		imshow(maskWinName, mask);

		std::cout << "error permitido: " << error << std::endl;

		if (0 < sleep)
			std::this_thread::sleep_for(std::chrono::seconds(1));

		dt = 0;
		// If the user presses a key, the cycle ends.
	} while (cv::waitKeyEx(30) < 0);

	imwrite("./media/LastFrame.png", inputFrame);

	// Close windows that were opened.
	cv::destroyWindow(inputWinName);
	cv::destroyWindow(maskWinName);

	std::cout << "\033[1;32m" << "End" << "\033[0m" << std::endl;
	return 0;
}
