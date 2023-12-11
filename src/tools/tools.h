//
// Created by dayan3847 on 15/10/23.
//

#ifndef TOOLS_H
#define TOOLS_H

#define IM_WIDTH 640 //!< El ancho de la imagen.
#define IM_HEIGHT 480 //!< Lo alto de la imagen.

#include "opencv2/opencv.hpp"
#include <fstream>

//#include <cstdio>
//#include <cmath>
//#include <Eigen/Dense>

namespace dayan
{
//	// Matriz de calibracion
//	cv::Mat k = (cv::Mat_<float>(3, 3)
//		<<
//		1377.8036814997304, 0, 400.02681782947193,
//		0, 1377.8036814997304, 300.096061319675721,
//		0, 0, 1
//	);
//
//	// Matriz de calibracion (inversa)
//	cv::Mat kInv = (cv::Mat_<float>(3, 3)
//		<<
//		0.000725792805918116, 0, -0.290336586554948,
//		0, 0.000725792805918116, -0.217807562390183,
//		0, 0, 1
//	);
//
//	// TODO: Obtener estos valores de un archivo de configuracion
//	float realR = 0.0199; //!< Radio real del circulo naranja (en metros)

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

	void readTimes(const std::string& timesFilename, std::vector<float>& times)
	{
		// Leer un archivo de texto
		std::ifstream infile(timesFilename);
		float time;
		while (infile >> time)
		{
			times.push_back(time);
//			times.push_back(time / 1000); // convertir a segundos
		}
	}

	void readDeltaTimes(const std::string& timesFilename, std::vector<float>& deltaTimes)
	{
		std::vector<float> times;
		readTimes(timesFilename, times);
		deltaTimes = std::vector<float>(times.size());
		deltaTimes[0] = 0;
		for (int i = 1; i < (int)times.size(); ++i)
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
		for (auto& contour: contours)
		{
			if (contourPointMinCount > (int)contour.size())
				continue;
			std::cout << "cantidad de puntos del contorno: " << contour.size() << std::endl;
			std::vector</*arturo::*/Point3s> pts;
			for (auto& contourPoint: contour)
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
		for (auto& c: circles)
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

	void drawCircle(const cv::Mat& inputFrame, Circle*& c)
	{
		cv::Point center(cvRound(c->h), cvRound(c->k));
		int radius = cvRound(c->r);
		cv::circle(inputFrame, center, radius, cv::Scalar(0, 0, 255),
				3, 8, 0);
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
				stream << mat.at<float>(i, j);
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

	void saveMatToFile(const cv::Mat& matrix, const std::string& filename)
	{
		cv::FileStorage fs(filename, cv::FileStorage::WRITE);

		if (!fs.isOpened())
		{
			std::cerr << "No se pudo abrir el archivo para escribir." << std::endl;
			return;
		}

		fs << "matrix" << matrix;
		fs.release();
	}

	void readMatFromFile(cv::Mat& matrix, const std::string& filename)
	{
		cv::FileStorage fs(filename, cv::FileStorage::READ);

		if (!fs.isOpened())
		{
			std::cerr << "No se pudo abrir el archivo para leer." << std::endl;
			return;
		}

		fs["matrix"] >> matrix;
		fs.release();
	}

	bool cholesky(const cv::Mat& mat, cv::Mat& cho)
	{
		int rows = mat.rows;
		int cols = mat.cols;
		if (rows != cols)
		{
			std::cout << "\033[31m" << "Error: Matrix is not square" << "\033[0m" << std::endl;
			return false;
		}
		cv::Mat temp;
		mat.copyTo(temp);
		bool success = cv::Cholesky(temp.ptr<float>(), temp.step, rows, nullptr, 0, 0);
//		bool success = cv::Cholesky(sqrtMatrix.ptr<float>(), sqrtMatrix.step1(), rows, nullptr, 0, cols);
		if (!success)
		{
			std::cout << "\033[31m" << "Error: Cholesky failed" << "\033[0m" << std::endl;
			return false;
		}
		cho = temp;
		return true;
	}
}

#endif //TOOLS_H
