#include <iostream>
#include <chrono>
#include <thread>
#include "opencv2/opencv.hpp"
#include "lib/arturo/functions.h"
#include "lib/arturo/Circle/Circle.h"
#include "lib/tools.h"


int main(int argc, char** argv)
{
	std::string filename;
	std::string colorModel;
	if (argc < 2)
	{
		filename = "./media/PelotaVerde.mkv";
		colorModel = "./colors/ColorVerde.png";
	}
	else
	{
		filename = argv[1];
		colorModel = argv[2];
	}

	// Input
	cv::VideoCapture inputVideoCapture;
	cv::Mat inputFrame, inputFrameLab;

	my_tools::getVideoCapture(filename, inputVideoCapture);

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

	cv::Mat mask, mean, iCov;
	bool first = true;
	int frameCount = 0;
	do
	{
		std::cout << "\033[1;32m" << "Frame: " << frameCount++ << "\033[0m" << std::endl;
		// We capture an image, and validate that the operation worked.
		inputVideoCapture >> inputFrame;
		if (inputFrame.empty())
			break;

		arturo::convertLab(inputFrame, inputFrameLab);

		// In the first iteration we initialize the images that we will use to store results.
		if (first)
		{
			arturo::convertLab(colorFrame, inputFrameLab);

			cv::Mat mMask = cv::Mat::ones(colorFrame.size(), CV_8UC1);
			arturo::MeaniCov(inputFrameLab, mMask, mean, iCov);

			cv::Size sz(inputFrame.cols, inputFrame.rows);
			mask = cv::Mat::ones(sz, CV_8U);

			first = false;
		}

		arturo::Umbraliza(inputFrameLab, mask, mean, iCov, umDist.val, umLuz.val);

		// Find Contours
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

		// Draw Contours
		for (int i = 0; i < contours.size(); i++)
		{
			cv::drawContours(inputFrame, contours, i,
				cv::Scalar(255, 255, 255), 2, 8,
				hierarchy, 0, cv::Point());
		}

		// Fit Circles
		std::vector<Circle> circles;
		for (auto& contour : contours)
		{
			if (contourPointMinCount > contour.size())
				continue;
			std::cout << "cantidad de puntos del contorno: " << contour.size() << std::endl;
			std::vector</*arturo::*/Point3s> pts;
			for (auto& contourPoint : contour)
			{
				/*arturo::*/Point3s p;
				p.x = contourPoint.x;
				p.y = contourPoint.y;
				p.z = 0;
				pts.push_back(p);
			}
			// Ransac Fit
			/*arturo::*/Circle c(pts);
			unsigned int nInl;
			float w = .6;
			float sigma = 1;
			float p = .99;
			float e = c.ransacFit(pts, nInl, w, sigma, p);
			if (0 > e)
				continue;
			if ((float)error < e)
				continue;
			std::cout << "e: " << e << " r: " << c.r << std::endl;
//			if (20 > c.r)
//				continue;
			circles.push_back(c);
		}

		// Draw Circles ( in red )
		for (auto& c : circles)
		{
			cv::Point center(cvRound(c.h), cvRound(c.k));
			int radius = cvRound(c.r);
			if (0 >= radius)
				continue;
			cv::circle(inputFrame, center, radius, cv::Scalar(0, 0, 255),
				3, 8, 0);
		}

		// Show images
		imshow(inputWinName, inputFrame);
		imshow(maskWinName, mask);

		std::cout << "error permitido: " << error << std::endl;

		if (0 < sleep)
			std::this_thread::sleep_for(std::chrono::seconds(1));

		// If the user presses a key, the cycle ends.
	} while (cv::waitKeyEx(30) < 0);

	imwrite("./media/LastFrame.png", inputFrame);

	// Close windows that were opened.
	cv::destroyWindow(inputWinName);
	cv::destroyWindow(maskWinName);

	std::cout << "\033[1;32m" << "End" << "\033[0m" << std::endl;
	return 0;
}
