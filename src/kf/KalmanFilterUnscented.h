//
// Created by dayan3847 on 20/11/23.
//

#ifndef KALMANFILTERUNSCENTED_H
#define KALMANFILTERUNSCENTED_H

#include "opencv2/opencv.hpp"
#include "KalmanFilter.h"

namespace dayan
{

	// Unscented Kalman Filter
	class KalmanFilterUnscented : public KalmanFilter
	{
	public:
		float alpha; //!< Parámetro alpha
		float beta; //!< Parámetro beta
		float kappa; //!< Parámetro kappa
		float lambda; //!< Parámetro lambda
		float theta; //!< Parámetro theta
		int _2n_1; //!< 2n+1
		float wm_0;
		float wc_0;
		float w_i;

		KalmanFilterUnscented(int n, int m)
				: KalmanFilter(n, m)
		{
			_2n_1 = 2 * n + 1;
			auto n_ = (float)n;
			// Inicialización de los parámetros
			alpha = 1;
			beta = 2;
			kappa = 1;
			auto alpha2 = alpha * alpha;
			lambda = alpha2 * (n_ + kappa) - n_;
			theta = sqrt(n_ + lambda);
			wm_0 = lambda / (n_ + lambda);
			wc_0 = wm_0 + (1 - alpha2 + beta);
			w_i = 1 / (2 * (n_ + lambda));
		}

		void predict_correct() override
		{
			this->update_A();
			Xp = A * X;
			Pp = A * P * A.t() + Q;

			cv::Mat sqrtP;
			bool success = dayan::cholesky(Pp, sqrtP);

			if (!success)
			{
				dayan::printMat(Pp);
				exit(1);
			}
			// Generación de los puntos sigma
			cv::Mat arrayX[_2n_1];
			cv::Mat mu;
			Xp.copyTo(mu);
			arrayX[0] = mu;
			for (int i = 1; i <= n; i++)
			{
				cv::Mat sqrtP_i = sqrtP.col(i - 1);
				cv::Mat lambdaXsqrtP_i = theta * sqrtP_i;
				arrayX[i] = mu + lambdaXsqrtP_i;
				arrayX[i + n] = mu - lambdaXsqrtP_i;
			}

			cv::Mat arrayZ[_2n_1];
			for (int i = 0; i < _2n_1; i++)
			{
				this->get_h(arrayX[i], arrayZ[i]);
			}

			// Promedio de las predicciones
			cv::Mat Zmean = wm_0 * arrayZ[0];
			for (int i = 1; i < _2n_1; i++)
			{
				Zmean += w_i * arrayZ[i];
			}

			cv::Mat Xdiff = arrayX[0] - mu;
			cv::Mat Zdiff = arrayZ[0] - Zmean;
			cv::Mat Zdiff_t = Zdiff.t();
			cv::Mat Smean = wc_0 * (Zdiff * Zdiff_t);
			cv::Mat Pmean = wc_0 * (Xdiff * Zdiff_t);
			for (int i = 1; i < _2n_1; i++)
			{
				Xdiff = arrayX[i] - mu;
				Zdiff = arrayZ[i] - Zmean;
				Zdiff_t = Zdiff.t();
				Smean += w_i * (Zdiff * Zdiff_t);
				Pmean += w_i * (Xdiff * Zdiff_t);
			}
			Smean += R;

			K = Pmean * Smean.inv();

			X = Xp + K * (Z - Zmean);
			P = Pp - (K * Smean * K.t());
			if (P.at<float>(0) < 0)
			{
				dayan::printMat(P, "P");
				dayan::printMat(Pp, "Pp");
				dayan::printMat(K, "K");
				dayan::printMat(Smean, "Smean");
				dayan::printMat(Zmean, "Zmean");
				dayan::printMat(Z, "Z");
				dayan::printMat(Xp, "Xp");
				dayan::printMat(X, "X");
				dayan::printMat(mu, "mu");
				dayan::printMat(A, "A");
				dayan::printMat(R, "R");
				dayan::printMat(Q, "Q");
			}
		}

	protected:
		void update_h_jacobians() override
		{
			this->get_h(this->Xp, this->h);
		}

		virtual void get_h(const cv::Mat& XX, cv::Mat& h)
		{

		}
	};

} // dayan

#endif //KALMANFILTERUNSCENTED_H
