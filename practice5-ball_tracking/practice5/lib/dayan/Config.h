//
// Created by dayan3847 on 13/11/23.
//

#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
#include "opencv2/opencv.hpp"
#include "tools.h"

namespace dayan
{
	class Config
	{
	private:
		Config(const std::string& data_path)
		{
			// Color
			std::string file_color = "./media/" + data_path + "/color.png";
			this->color = cv::imread(file_color);
			// Radio
			std::string file_diameter_cm = "./media/" + data_path + "/diameter_cm.txt";
			float diameter_cm;
			std::ifstream file_diameter_cm_stream(file_diameter_cm);
			file_diameter_cm_stream >> diameter_cm;
			float radio_cm = diameter_cm / 2;
			this->radio = radio_cm / 100;
			// Delta Times
			std::string file_times = "./media/" + data_path + "/times.txt";
			dayan::readDeltaTimes(file_times, this->dTimes);
			// Video
			std::string file_video = "./media/" + data_path + "/video.mkv";
			dayan::getVideoCapture(file_video, this->video);
			// K (camera matrix)
			std::string file_k = "./media/" + data_path + "/k.yaml";
			dayan::readMatFromFile(this->k, file_k);
			cv::invert(this->k, this->kInv);
			// Crear un vector vacio de Z (mediciones)
			this->list_Z = std::vector<cv::Mat>();
		}
		~Config() = default;
		static Config* instance;
	public:
		static Config* getInstance(const std::string& data_path = "")
		{
			if (!instance)
			{
				instance = new Config(data_path);
			}
			return instance;
		}

		cv::Mat color; //!< Modelo de color
		float radio; //!< Radio Real (en metros)
		std::vector<int> dTimes; //!< Vector de tiempos entre frames (deltaTimes)
		cv::VideoCapture video; //!< Video de entrada
		cv::Mat k; //!< Matriz de calibracion
		cv::Mat kInv; //!< Matriz de calibracion (inversa)
		// Crear un vector vacio de Z (mediciones)
		std::vector<cv::Mat> list_Z;
	};

	Config* Config::instance = nullptr;

} // dayan

#endif //CONFIG_H
