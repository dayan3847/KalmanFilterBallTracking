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

	// Matriz de calibracion
	cv::Mat k = (cv::Mat_<float>(3, 3)
		<<
		1377.8036814997304, 0, 400.02681782947193,
		0, 1377.8036814997304, 300.096061319675721,
		0, 0, 1
	);

	// Matriz de calibracion (inversa)
	cv::Mat kInv = (cv::Mat_<float>(3, 3)
		<<
		0.000725792805918116, 0, -0.290336586554948,
		0, 0.000725792805918116, -0.217807562390183,
		0, 0, 1
	);

	float realR = .03;

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
		std::vector<cv::Mat>& list_Z,
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


		cv::Mat prevZ = list_Z.empty() ? cv::Mat::zeros(5, 1, CV_32F) : list_Z.back();
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
		list_Z.push_back(Z);
	}

	void drawCircle(const cv::Mat& inputFrame, Circle*& c)
	{
		cv::Point center(cvRound(c->h), cvRound(c->k));
		int radius = cvRound(c->r);
		cv::circle(inputFrame, center, radius, cv::Scalar(0, 0, 255),
			3, 8, 0);
	}

	// obtener un estado inicial a partir de una medicion
	void getX(const cv::Mat& ZZ, cv::Mat& XX)
	{
		auto x = ZZ.at<float>(0);
		auto y = ZZ.at<float>(1);
		auto r = ZZ.at<float>(4);

		auto Z = realR / r;
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
			realR / Z
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
			0, 0, -realR / Z2, 0, 0, 0
		);
	}

	// pintar el circulo a partir de una medicion
	void drawCircleByZ(
		const cv::Mat& inputFrame,
		const cv::Mat& ZZ,
		const cv::Scalar& color
	)
	{
		// Centro en metros
		cv::Mat centerMe = (cv::Mat_<float>(3, 1)
			<<
			ZZ.at<float>(0),
			ZZ.at<float>(1),
			1
		);
		// Centro en pixeles
		cv::Mat centerPx = k * centerMe;
		centerPx = centerPx / centerPx.at<float>(2);
		cv::Point center(cvRound(centerPx.at<float>(0)), cvRound(centerPx.at<float>(1)));
		// Radio en metros
		float radiusMe = ZZ.at<float>(4);
		// Radio en pixeles
		float radiusPx = radiusMe * k.at<float>(0, 0);
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
		float X = XX.at<float>(0);
		float Y = XX.at<float>(1);
		float Z = XX.at<float>(2);

		cv::Mat ZZ = (cv::Mat_<float>(5, 1)
			<<
			X / Z,
			Y / Z,
			0,
			0,
			realR / Z
		);

		drawCircleByZ(inputFrame, ZZ, color);
	}

	std::string matToString(const cv::Mat& mat, bool tab = false)
	{
		char s = tab ? '\t' : ' ';
		std::ostringstream stream;
		std::string matString;
		for (int i = 0; i < mat.rows; i++)
		{
			for (int j = 0; j < mat.cols; j++)
			{
				stream << mat.at<double>(i, j);
				matString += stream.str();
				if (j < mat.cols - 1)
					matString += s;
				stream.str("");
			}
			matString += "\n";
		}
		return matString;
	}

	void printMat(const cv::Mat& mat, const std::string& name = "Mat:")
	{
		std::cout << name << std::endl;
		std::string matString = matToString(mat, true);
		std::cout << matString << std::endl;
	}
}

#endif //TOOLS_H
