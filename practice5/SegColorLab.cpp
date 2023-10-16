#include "opencv2/opencv.hpp"
#include <iostream>
#include "lib/arturo/lib_arturo.h"
#include "lib/tools.h"


int main(int argc, char** argv)
{
	// Input
	cv::VideoCapture inputVideoCapture;
	cv::Mat inputFrame, inputFrameLab;

	std::string filename = "./media/PelotaVerde.mkv";
	my_tools::getVideoCapture(filename, inputVideoCapture);

	std::string colorModel = "./media/ColorVerde.png";
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
		//Capturamos una imagen, y validamos que haya funcionado la operacion.
		inputVideoCapture >> inputFrame;
		if (inputFrame.empty())
			break;

		arturo::convertLab(inputFrame, inputFrameLab);

		//En la primera iteración inicializamos las imagenes que usaremos para
		//almacenar resultados.
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
			cv::Scalar color = cv::Scalar(255, 255, 255);
			cv::drawContours(inputFrame, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
		}

		// Fit Circles
//		std::vector<arturo::Circle> circles;
//		arturo::ransacFit(drawing, circles, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1);
//
//		// Draw Circles
//		for (int i = 0; i < circles.size(); i++)
//		{
//			cv::Point center(cvRound(circles[i].x), cvRound(circles[i].y));
//			int radius = cvRound(circles[i].r);
//			cv::circle(inputFrame, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
//		}

		// Aqui tienen que hacer lo necesario para que a la imagen segmentada
		// que está almacenada en Mask se extraga el contorno y se ajuste un
		// circulo a esta. Para esto, usar la función findContours.
		//
		// Algo asó como lo que sigue:
		//
		// findContours(output, Contornos, RETR_EXTERNAL, CHAIN_APPROX_NONE);
		//
		// Donde Contorns es como sigue:
		//
		// vector< vector<Point> >Contornos;
		//
		// Cada elemento de Contornos es una vector de coordenadas y cada uno
		// de estos vectores corresponde a un objeto segmentado en la imagen.
		// Estos vectores hay que ajustarlos a un circulo usando la clase
		// Circle y ransacFit.

		imshow(inputWinName, inputFrame);
		imshow(maskWinName, mask);

		//Si el usuario oprime una tecla, termina el ciclo.
	} while (cv::waitKeyEx(30) < 0);

	imwrite("./media/LastFrame.png", inputFrame);

	//Cierra ventanas que fueron abiertas.
	cv::destroyWindow(inputWinName);
	cv::destroyWindow(maskWinName);

	std::cout << "\033[1;32m" << "End" << "\033[0m" << std::endl;
	return 0;
}