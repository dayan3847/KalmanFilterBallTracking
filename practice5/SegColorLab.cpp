#include "opencv2/opencv.hpp"
#include <iostream>
#include "lib/arturo/functions.h"
#include "lib/arturo/Circle.h"
#include "lib/tools.h"


int main(int argc, char** argv)
{

	std::string filename;
	std::string colorModel;
	if (argc < 2)
	{
		filename = "./media/PelotaVerde.mkv";
		colorModel = "./media/ColorVerde.png";
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

	int dSlidePos = 16;
	arturo::barData umDist(40. / SLIDE_MAX, 10);
	cv::createTrackbar("umDist", inputWinName, &dSlidePos, SLIDE_MAX, arturo::umDistChange,
		(void*)&umDist);
	arturo::umDistChange(SLIDE_MAX, (void*)&umDist);

	int lSlidePos = 16;
	arturo::barData umLuz(100. / SLIDE_MAX, 0);
	cv::createTrackbar("umLuz", inputWinName, &lSlidePos, SLIDE_MAX, arturo::umLuzChange,
		(void*)&umLuz);
	arturo::umLuzChange(0, (void*)&umLuz);


	cv::Mat mask, mean, iCov;
	bool first = true;
	do
	{
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
		std::vector<arturo::Circle> circles;
		for (auto& contour : contours)
		{
			std::vector<arturo::Point3s> pts;
			for (auto& contourPoint : contour)
			{
				arturo::Point3s p;
				p.x = contourPoint.x;
				p.y = contourPoint.y;
				p.z = 0;
				pts.push_back(p);
			}
			arturo::Circle c(pts);
			circles.push_back(c);
		}

		// Draw Circles ( in red )
		for (auto& c : circles)
		{
			cv::Point center(cvRound(c.h), cvRound(c.k));
			int radius = cvRound(c.r);
			if (0 >= radius || 1000 < radius)
				continue;
			cv::circle(inputFrame, center, radius, cv::Scalar(0, 0, 255),
				3, 8, 0);
		}

		// Show images
		imshow(inputWinName, inputFrame);
		imshow(maskWinName, mask);

		// If the user presses a key, the cycle ends.
	} while (cv::waitKeyEx(30) < 0);

	imwrite("./media/LastFrame.png", inputFrame);

	// Close windows that were opened.
	cv::destroyWindow(inputWinName);
	cv::destroyWindow(maskWinName);

	std::cout << "\033[1;32m" << "End" << "\033[0m" << std::endl;
	return 0;
}
