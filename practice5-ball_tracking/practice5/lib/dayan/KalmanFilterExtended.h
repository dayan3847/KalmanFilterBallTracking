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
		void correct() override
		{
			this->update_h_jacobians();
			this->update_K();
			X = Xp + K * (Z - h);
			cv::Mat I = cv::Mat::eye(6, 6, CV_32F);
			P = (I - K * H) * Pp;
		}
	protected:
		// Actualiza h y sus jacobianos (H y J[si es implicito])
		virtual void update_h_jacobians()
		{

		}
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
