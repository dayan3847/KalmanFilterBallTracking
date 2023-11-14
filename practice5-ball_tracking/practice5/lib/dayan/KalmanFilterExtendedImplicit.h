//
// Created by dayan3847 on 13/11/23.
//

#ifndef KALMANFILTEREXTENDEDIMPLICIT_H
#define KALMANFILTEREXTENDEDIMPLICIT_H

#include "KalmanFilterExtended.h"
#include "functions.h"

namespace dayan
{
	// Implicit Extended Kalman Filter
	class KalmanFilterExtendedImplicit : public dayan::KalmanFilterExtended
	{
	public:
		cv::Mat J; //!< Jacobian matrix h respecto a Z

		void correct(const cv::Mat& Z) override
		{
//			dayan::getJ(Xp, J);
			R = J * R * J.t();


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

#endif //KALMANFILTEREXTENDEDIMPLICIT_H
