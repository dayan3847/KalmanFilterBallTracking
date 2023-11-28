//
// Created by dayan3847 on 13/11/23.
//

#ifndef KALMANFILTEREXTENDEDIMPLICIT_H
#define KALMANFILTEREXTENDEDIMPLICIT_H

#include "KalmanFilterExtended.h"

namespace dayan
{
	// Implicit Extended Kalman Filter
	class KalmanFilterExtendedImplicit : public KalmanFilterExtended
	{
	public:
		KalmanFilterExtendedImplicit(int n, int m)
				: KalmanFilterExtended(n, m)
		{
		}

		cv::Mat J; //!< Jacobian matrix h respecto a Z

		void correct() override
		{
			this->update_h_jacobians();
			auto R_ = J * R * J.t();
//			this->update_K();
			cv::Mat Ht = H.t();
			cv::Mat HxPpxHtmR = H * Pp * Ht + R_;
			cv::Mat inv_HxPpxHtmR;
			cv::invert(HxPpxHtmR, inv_HxPpxHtmR, cv::DECOMP_LU);
			K = Pp * Ht * inv_HxPpxHtmR;

			X = Xp + K * (-h);
			P = (I - K * H) * Pp;
		}
	};
} // dayan

#endif //KALMANFILTEREXTENDEDIMPLICIT_H
