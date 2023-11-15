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
			R = J * R * J.t();
			// TODO: completar
		}
	};
} // dayan

#endif //KALMANFILTEREXTENDEDIMPLICIT_H
