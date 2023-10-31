//
// Created by dayan3847 on 15/10/23.
//

#ifndef TOOLS_H
#define TOOLS_H

#define IM_WIDTH 640 //!< El ancho de la imagen.
#define IM_HEIGHT 480 //!< Lo alto de la imagen.

#include "opencv2/opencv.hpp"
#include <fstream>


namespace dayan
{
	bool isInteger(const std::string& s)
	{
		try
		{
			std::stoi(s);
			return true;
		}
		catch (const std::invalid_argument& ia)
		{
			return false;
		}
	}

	void getVideoCapture(const std::string& filename, cv::VideoCapture& videoCapture)
	{
		videoCapture = dayan::isInteger(filename)
					   ? cv::VideoCapture(std::stoi(filename))
					   : cv::VideoCapture(filename);
		if (!videoCapture.isOpened())
		{
			std::cerr << "Error opening video file" << std::endl;
			exit(1);
		}

		double oW = videoCapture.get(cv::CAP_PROP_FRAME_WIDTH);
		double oH = videoCapture.get(cv::CAP_PROP_FRAME_HEIGHT);
		// We define the size of the images to be captured.
		videoCapture.set(cv::CAP_PROP_FRAME_WIDTH, IM_WIDTH);
		videoCapture.set(cv::CAP_PROP_FRAME_HEIGHT, IM_HEIGHT);
		std::cout << "Changed frame size from ("
				  << oW << " x " << oH << ") to (" << IM_WIDTH << " x " << IM_HEIGHT << ")" << std::endl;
	}

	void readTimes(const std::string& timesFilename, std::vector<int>& times)
	{
		// Leer un archivo de texto
		std::ifstream infile(timesFilename);
		int time;
		while (infile >> time)
		{
			times.push_back(time);
		}
	}

	void readDeltaTimes(const std::string& timesFilename, std::vector<int>& deltaTimes)
	{
		std::vector<int> times;
		readTimes(timesFilename, times);
		deltaTimes = std::vector<int>(times.size());
		deltaTimes[0] = 0;
		for (int i = 1; i < times.size(); ++i)
		{
			deltaTimes[i] = times[i] - times[i - 1];
		}
	}

	// identificar el circulo a partir de
	void makeMeasurement(
		cv::Mat& inputFrameLab,
		cv::Mat& mask,
		cv::Mat& mean,
		cv::Mat& iCov,
		float const& umDistVal,
		float const& umLuzVal,
		int const& contourPointMinCount,
		int const& error,
		Circle*& circle
	)
	{
		arturo::Umbraliza(inputFrameLab, mask, mean, iCov, umDistVal, umLuzVal);

		// Find Contours
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

		// Draw Contours
//		for (int i = 0; i < contours.size(); i++)
//		{
//			cv::drawContours(inputFrame, contours, i,
//				cv::Scalar(255, 255, 255), 2, 8,
//				hierarchy, 0, cv::Point());
//		}

		// Fit Circles
		std::vector<Circle*> circles;
		for (auto& contour : contours)
		{
			if (contourPointMinCount > contour.size())
				continue;
			std::cout << "cantidad de puntos del contorno: " << contour.size() << std::endl;
			std::vector</*arturo::*/Point3s> pts;
			for (auto& contourPoint : contour)
			{
				Point3s p;
				p.x = contourPoint.x;
				p.y = contourPoint.y;
				p.z = 0;
				pts.push_back(p);
			}
			// Ransac Fit
			auto* c = new Circle(pts);
			unsigned int nInl;
			float w = .6;
			float sigma = 1;
			float p = .99;
			float e = c->ransacFit(pts, nInl, w, sigma, p);
			if (0 > e)
				continue;
			if ((float)error < e)
				continue;
			std::cout << "e: " << e << " r: " << c->r << std::endl;
//			if (20 > c.r)
//				continue;
			circles.push_back(c);
		}

		// Draw Circles ( in red )
		for (auto& c : circles)
		{
//			cv::Point center(cvRound(c.h), cvRound(c.k));
			int radius = cvRound(c->r);
			if (0 >= radius)
				continue;
			circle = c;
			return;
//			cv::circle(inputFrame, center, radius, cv::Scalar(0, 0, 255),
//				3, 8, 0);
		}
	}

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
		cv::Mat& Z
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

		// Matriz de calibracion
//		cv::Mat k = (cv::Mat_<float>(3, 3)
//			<<
//			1377.8036814997304, 0, 400.02681782947193,
//			0, 1377.8036814997304, 300.096061319675721,
//			0, 0, 1
//		);
		// Matriz de calibracion (inversa)
		cv::Mat kInv = (cv::Mat_<float>(3, 3)
			<<
			0.000725792805918116, 0, -0.290336586554948,
			0, 0.000725792805918116, -0.217807562390183,
			0, 0, 1
		);
		// Centro en pixeles
		cv::Mat centerPx = (cv::Mat_<float>(3, 1)
			<<
			circle->h,
			circle->k,
			1
		);
		// Centro en metros
		cv::Mat centerMe = kInv * centerPx;
		centerMe = centerMe / centerMe.at<float>(2);
		// Radio en pixeles
		float radiusPx = circle->r;
		// Radio en metros
		float radiusMe = radiusPx * kInv.at<float>(0, 0);
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

			Z = (cv::Mat_<float>(3, 1)
			<<
			centerMe.at<float>(0),
			centerMe.at<float>(1),
			radiusMe
		);
	}
}

#endif //TOOLS_H
