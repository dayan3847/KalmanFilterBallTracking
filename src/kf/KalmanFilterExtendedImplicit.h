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
		cv::Mat J; //!< Jacobian matrix h respecto a Z

		void correct() override
		{
			this->update_h_jacobians();
			R = J * R * J.t();
			this->update_K();
			X = Xp + K * (-h);
			cv::Mat I = cv::Mat::eye(6, 6, CV_32F);
			P = (I - K * H) * Pp;
		}
	};
} // dayan

#endif //KALMANFILTEREXTENDEDIMPLICIT_H