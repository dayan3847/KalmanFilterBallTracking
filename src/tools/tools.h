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
#include <Eigen/Dense>

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
		for (auto& contour : contours)
		{
			if (contourPointMinCount > (int)contour.size())
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

	// Calculate the square root of a matrix.
	void sqrtMat(const cv::Mat& matrix, cv::Mat& sqrtMatrix)
	{
		cv::Mat eigenValues;
		cv::Mat eigenVectors;
		cv::eigen(matrix, eigenValues, eigenVectors);
		cv::sqrt(eigenValues, eigenValues);
		sqrtMatrix = eigenVectors * cv::Mat::diag(eigenValues) * eigenVectors.inv();
	}


	void sqrtMatMario(const cv::Mat& matrix, cv::Mat& cvMatrix)
	{
		std::cout << matrix << std::endl << std::endl;

		// Convert a Mat into a Eigen matrix using the constructor.
		Eigen::MatrixXf sqrtCovMatrix(matrix.rows, matrix.cols);

		// Pass data into the Eigen matrix.
		for (int i = 0; i < matrix.rows; i++)
			for (int j = 0; j < matrix.cols; j++)
				sqrtCovMatrix(i, j) = matrix.at<float>(i, j);

		std::cout << sqrtCovMatrix << std::endl << std::endl;

		// Calculate Cholesky decomposition.
		Eigen::LLT<Eigen::MatrixXf> lltOfA(sqrtCovMatrix);

		Eigen::MatrixXf L;
		// Check if the decomposition was successful.
		if (lltOfA.info() == Eigen::Success)
		{
			// Obtain the lower triangular matrix L from the Cholesky decomposition.
			L = lltOfA.matrixL();
		}
		else
		{
			std::cerr << "Cholesky's decomposition was not successful." << std::endl;
		}

// Convert a matrix from Eigen library to opencv.
//		cvMatrix(L.rows(), L.cols(), CV_32F, L.data());
		cvMatrix = cv::Mat(L.rows(), L.cols(), CV_32F, L.data());

		std::cout << cvMatrix << std::endl << std::endl;
	}
}

#endif //TOOLS_H
