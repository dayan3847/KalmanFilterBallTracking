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
			R = J * R * J.t();
			this->update_K();
			X = Xp + K * (-h);
			P = (I - K * H) * Pp;
		}
	};
} // dayan

#endif //KALMANFILTEREXTENDEDIMPLICIT_H
