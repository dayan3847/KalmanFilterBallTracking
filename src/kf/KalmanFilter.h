//
// Created by dayan3847 on 13/11/23.
//

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "opencv2/opencv.hpp"
#include "../tools/tools.h"

namespace dayan
{

	// Kalman Filter
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

		cv::Mat Z;
		cv::Mat h;

		cv::Mat I; //!< identity matrix

		// catidad de parametros del estado
		int n;
		// catidad de parametros de la medida
		int m;

		KalmanFilter(int n, int m)
		{
			this->n = n;
			this->m = m;
			Z = cv::Mat::zeros(m, 1, CV_32F);
		}

		void predict(const float& dt)
		{
			this->update_A(dt);
			Xp = A * X;
			Pp = A * P * A.t() + Q;
			if (Pp.at<float>(0) < 0)
			{
				dayan::printMat(Pp, "Pp");
				dayan::printMat(A, "A");
				dayan::printMat(P, "P");
				dayan::printMat(Q, "Q");
			}
		}

		virtual void correct()
		{
		}

		virtual void predict_correct(const float& dt)
		{
			this->predict(dt);
			this->correct();
		}

		virtual void init_X()
		{
		}

	protected:
		virtual void update_A(const float& dt)
		{
		}

		// Actualiza h y sus jacobianos (H y J[si es implicito])
		virtual void update_h_jacobians()
		{

		}
	};

} // dayan

#endif //KALMANFILTER_H
