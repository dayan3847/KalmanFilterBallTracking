//
// Created by dayan3847 on 13/11/23.
//

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "opencv2/opencv.hpp"
#include "tools.h"

namespace dayan
{

	class KalmanFilter
	{
	public:
		// statePre
		cv::Mat Xp; //!< predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k)
		// statePost
		cv::Mat X; //!< corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
		// transitionMatrix
		cv::Mat A; //!< state transition matrix (A)
		// measurementMatrix
		cv::Mat H;  //!< measurement matrix (H)
		// processNoiseCov
		cv::Mat Q; //!< process noise covariance matrix (Q)
		// measurementNoiseCov
		cv::Mat R; //!< measurement noise covariance matrix (R)
		// errorCovPre
		cv::Mat Pp; //!< priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)*/
		// gain
		cv::Mat K; //!< Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)
		// errorCovPost
		cv::Mat P; //!< posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k)

		void predict()
		{
			Xp = A * X;
			Pp = A * P * A.t() + Q;
		}

		virtual void correct(const cv::Mat& Z)
		{
			// TODO: Implement
		}

		void predict_correct(const cv::Mat& Z)
		{
			this->predict();
			this->correct(Z);
		}
	};

} // dayan

#endif //KALMANFILTER_H
