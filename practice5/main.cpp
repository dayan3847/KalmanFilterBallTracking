#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

int main(int argc, char **argv) {
    std::string filename = "./media/PelotaVerde.mkv";
    cv::VideoCapture videoCapture = cv::VideoCapture(filename);
    if (!videoCapture.isOpened()) {
        std::cerr << "Error opening video file" << std::endl;
        exit(1);
    }

    std::cout << "\033[1;32m" << "End" << "\033[0m" << std::endl;
    return 0;
}
