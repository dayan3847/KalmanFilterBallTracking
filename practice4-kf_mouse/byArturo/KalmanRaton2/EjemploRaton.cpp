#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/videoio.hpp"

#define drawCross( center, color, d )                                 \
line( img, Point( center.x - d, center.y - d ), Point( center.x + d, center.y + d ), color, 2, LINE_AA, 0); \
line( img, Point( center.x + d, center.y - d ), Point( center.x - d, center.y + d ), color, 2, LINE_AA, 0 )

using namespace cv;
using namespace std;


void on_mouseEvent (int event, int x, int y, int flags, void *p)
{
	 vector < Point > *mD = (vector < Point > *)p;

	 switch (event)
	 {
	   case EVENT_MOUSEMOVE:
			(*mD).insert ((*mD).begin (), Point (x, y));
			break;
	 }
}


int main ()
{

	 KalmanFilter KF (4, 2, 0);
	 Point mousePos;
	 Mat img (600, 800, CV_8UC3);
	 vector < Point > mousev, kalmanv;
	 double dtm, dts; //delta time in milliseconda and seconds.
	 unsigned int n;
	 char val;

    dtm = 10;
    dts = dtm * 10e-3;

// intialization of KF...
// [ 1 0 dts 0          Xn = Xn-1
//   0 1 0  dts
//   0 0  1  0
//   0 0  0  1
	 KF.transitionMatrix =
			(Mat_ < float >(4, 4) << 
			1, 0, dts, 0,
			0, 1, 0, dts,
			0, 0, 1, 0,
			0, 0, 0, 1);

	 Mat_ < float >measurement (2, 1);
	 
	 measurement.setTo (Scalar (0));

    //Inicializamo el estado
	 KF.statePre.at < float >(0) = mousePos.x;// x_{t-1}
	 KF.statePre.at < float >(1) = mousePos.y;
	 KF.statePre.at < float >(2) = 0;
	 KF.statePre.at < float >(3) = 0;

	 //Definimos la matriz de medición.
	 setIdentity (KF.measurementMatrix);
    KF.measurementMatrix *= 1;


    //Inicializamos las matrices de ruido.
	 setIdentity (KF.processNoiseCov, Scalar::all (1e-4));

	 setIdentity (KF.measurementNoiseCov, Scalar::all (10));
	 setIdentity (KF.errorCovPost, Scalar::all (.1));




	 cout << "ProcessNoiseCov : " << KF.processNoiseCov.rows << ", " << KF.processNoiseCov.cols <<endl;
	 cout << "measurementNoiseCov : " << KF.measurementNoiseCov.rows << ", " << KF.measurementNoiseCov.cols <<endl;
	 cout << "errorCovPost : " << KF.errorCovPost.rows << ", " << KF.errorCovPost.cols <<endl;
	
// Image to show mouse tracking
	 mousev.clear ();
	 kalmanv.clear ();
	 namedWindow ("mouse kalman", 1);
	 imshow ("mouse kalman", img);
	 waitKey (1);
	 setMouseCallback ("mouse kalman", on_mouseEvent, (void *) &mousev);

    n = 0;
	 while (1)
	 {
			if (mousev.size () > 1)
			{
				 // First predict, to update the internal statePre variable
				 // Primero, usamos una predicción para actualizar el estado del estado (statePre).
				 Mat prediction = KF.predict ();
				 Point predictPt (prediction.at < float >(0),
													prediction.at < float >(1));

				 // Get mouse point
				 // Obtenemos una medición del puntero del programa.
				 measurement (0) = mousev[0].x;
				 measurement (1) = mousev[0].y;
			}

				 // Fase de actualización
				 cout << "--------------------------------------------------------------------------------" << endl << n << endl
			      << "Measurement : " << measurement.t() << endl
				  << "statePre: " << KF.statePre.t() << endl;

            // Actualiza el estado a partir de la última medición.
            // estimated contiene la medición que corresponde a dicho estado.
				 Mat estimated = KF.correct (measurement);
				 cout << "Estimated.shape : (" << estimated.rows << ", " << estimated.cols << ")" << endl;

				  cout << "statePost: " << KF.statePost.t() << endl
				  << "H: " << KF.measurementMatrix << endl 
				  << "errorCovPost: " << KF.errorCovPost <<endl 
				  << "Gain: " << KF.gain <<endl << endl;


             //Grafica la trayectoria del raton en cyan, y la trayectoria estimada en rojo. 
				 Point statePt (estimated.at < float >(0), estimated.at < float >(1));
				 Point measPt (measurement (0), measurement (1));
				 // plot points
				 imshow ("mouse kalman", img);
				 img = Scalar::all (0);

				 mousev.push_back (measPt);
				 kalmanv.push_back (statePt);
				 drawCross (statePt, Scalar (255, 155, 0), 5);
				 drawCross (measPt, Scalar (0, 0, 255), 5);

				 for (uint i = 0; i < mousev.size () - 1; i++)
						line (img, mousev[i], mousev[i + 1], Scalar (255, 255, 0), 1);

				 for (uint i = 0; i < kalmanv.size () - 1; i++)
						line (img, kalmanv[i], kalmanv[i + 1], Scalar (0, 155, 255), 1);
			
			val = waitKey ((int)dtm);
			if (val == 27)
			    break;
			n++;
	 }

	 return 0;
}
