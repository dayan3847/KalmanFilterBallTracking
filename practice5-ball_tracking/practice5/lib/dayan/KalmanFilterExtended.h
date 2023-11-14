//
// Created by dayan3847 on 13/11/23.
//

#ifndef KALMANFILTEREXTENDED_H
#define KALMANFILTEREXTENDED_H

#include "opencv2/opencv.hpp"
#include "tools.h"
#include "KalmanFilter.h"

namespace dayan
{

	class KalmanFilterExtended : public dayan::KalmanFilter
	{
	public:

		void correct(const cv::Mat& Z) override
		{
			cv::Mat h;
			dayan::getH(Xp, h, H);
			cv::Mat Ht = H.t();
			cv::Mat HxPpxHtmR = H * Pp * Ht + R;
			cv::Mat inv_HxPpxHtmR;
			cv::invert(HxPpxHtmR, inv_HxPpxHtmR, cv::DECOMP_LU);
			K = Pp * Ht * inv_HxPpxHtmR;
			X = Xp + K * (Z - h);
			cv::Mat I = cv::Mat::eye(6, 6, CV_32F);
			P = (I - K * H) * Pp;
		}
	};

} // dayan

#endif //KALMANFILTEREXTENDED_H
