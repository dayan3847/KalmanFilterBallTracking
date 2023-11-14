//
// Created by dayan3847 on 13/11/23.
//

#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
#include <fstream>
#include "opencv2/opencv.hpp"
#include "tools.h"

namespace dayan
{

	class Config
	{
//	private:
//		static bool initialized;
	public:
		static cv::Mat color; //!< Modelo de color
		static float radio; //!< Radio Real (en metros)
		static std::vector<int> dTimes; //!< Vector de tiempos entre frames (deltaTimes)
		static cv::VideoCapture video; //!< Video de entrada
		static cv::Mat k; //!< Matriz de calibracion
		static cv::Mat kInv; //!< Matriz de calibracion (inversa)

		static void loadData(const std::string& data_path)
		{
//			if (initialized)
//			{
//				std::cerr << "Config already initialized" << std::endl;
//				exit(1);
//			}

			// Color
			std::string file_color = "./media/" + data_path + "/color.png";
			Config::color = cv::imread(file_color);
			// Radio
			std::string file_diameter_cm = "./media/" + data_path + "/diameter_cm.txt";
			float diameter_cm;
			std::ifstream file_diameter_cm_stream(file_diameter_cm);
			file_diameter_cm_stream >> diameter_cm;
			float radio_cm = diameter_cm / 2;
			Config::radio = radio_cm / 100;
			// Delta Times
			std::string file_times = "./media/" + data_path + "/times.txt";
			dayan::readDeltaTimes(file_times, dTimes);
			// Video
			std::string file_video = "./media/" + data_path + "/video.mkv";
			dayan::getVideoCapture(file_video, video);
			// K (camera matrix)
			std::string file_k = "./media/" + data_path + "/k.yaml";
			dayan::readMatFromFile(k, file_k);
			cv::invert(k, kInv);

//			initialized = true;
		}

	};

} // dayan

#endif //CONFIG_H
