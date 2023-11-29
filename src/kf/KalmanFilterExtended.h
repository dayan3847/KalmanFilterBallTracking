//
// Created by dayan3847 on 13/11/23.
//

#ifndef KALMANFILTEREXTENDED_H
#define KALMANFILTEREXTENDED_H

#include "opencv2/opencv.hpp"
#include "KalmanFilter.h"

namespace dayan
{
	// Extended Kalman Filter
	class KalmanFilterExtended : public KalmanFilter
	{
	public:
		KalmanFilterExtended(int n, int m)
			: KalmanFilter(n, m)
		{
			I = cv::Mat::eye(n, n, CV_32F);
		}
		void correct() override
		{
			this->update_h_jacobians();
			this->update_K();
			X = Xp + K * (Z - h);
			P = (I - K * H) * Pp;
		}
	protected:
		// Actualiza K
		void update_K()
		{
			cv::Mat Ht = H.t();
			cv::Mat HxPpxHtmR = H * Pp * Ht + R;
			cv::Mat inv_HxPpxHtmR;
			cv::invert(HxPpxHtmR, inv_HxPpxHtmR, cv::DECOMP_LU);
			K = Pp * Ht * inv_HxPpxHtmR;
		}
	};

} // dayan

#endif //KALMANFILTEREXTENDED_H
