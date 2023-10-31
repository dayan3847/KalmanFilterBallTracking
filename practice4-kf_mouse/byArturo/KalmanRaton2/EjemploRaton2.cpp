#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <sys/timeb.h>

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

	 KalmanFilter KF (6, 2, 0);
	 Point mousePos;
	 Mat img (600, 800, CV_8UC3);
	 vector < Point > mousev, kalmanv;
	 double dtm, dts, dts2; //delta time in milliseconda and seconds.
	 unsigned int n;
	 char val;
	 struct timespec start, end;
	 double startns, endns, realDts;

    dtm = 13.483;
    dts = dtm * 1e-3;
    dts2 = dts * dts;

// intialization of KF...
// [ 1 0  dts 0   0.5*dts^2 0          Xn = Xn-1
//   0 1  0   dts 0         0.5*dts^2 
//   0 0  1   0   dts       0
//   0 0  0   1   0         dts
//   0 0  0   0   1         0
//   0 0  0   0   0         1
	 KF.transitionMatrix =
			(Mat_ < float >(6, 6) << 1,  0,  dts,   0,   0.5*dts2,        0,\
			                         0,  1,  0,   dts,          0, 0.5*dts2,\
			                         0,  0,  1,     0,        dts,        0,\
			                         0,  0,  0,     1,          0,      dts,\
			                         0,  0,  0,     0,          1,        0,\
			                         0,  0,  0,     0,          0,        1);

	 Mat_ < float >measurement (2, 1);
	 
	 measurement.setTo (Scalar (0));

	 KF.statePre.at < float >(0) = mousePos.x;
	 KF.statePre.at < float >(1) = mousePos.y;
	 KF.statePre.at < float >(2) = 0;
	 KF.statePre.at < float >(3) = 0;
	 setIdentity (KF.measurementMatrix);
	 setIdentity (KF.processNoiseCov, Scalar::all (1e-4));
	 setIdentity (KF.measurementNoiseCov, Scalar::all (10));
	 setIdentity (KF.errorCovPost, Scalar::all (.1));
 //   KF.measurementMatrix *= 0.5;

    cout << endl << endl
         << "Measurement Matrix Dimensions: (" 
         << KF.measurementMatrix.rows << ", " 
         << KF.measurementMatrix.cols << ")" << endl
         << "Measurement Matrix Dimensions:" << endl
         << KF.measurementMatrix 
         << endl << endl
         << "Transition Matrix Dimensions: (" 
         << KF.transitionMatrix.rows << ", " 
         << KF.transitionMatrix.cols << ")" << endl
         << "Transition Matrix Dimensions:" << endl
         << KF.transitionMatrix 
         << endl << endl;
	
// Image to show mouse tracking
	 mousev.clear ();
	 kalmanv.clear ();
	 namedWindow ("mouse kalman", 1);
	 imshow ("mouse kalman", img);
	 waitKey (1);
	 setMouseCallback ("mouse kalman", on_mouseEvent, (void *) &mousev);

    n = 0;
	 clock_gettime (CLOCK_REALTIME, &end);
    endns = (double)(end.tv_sec)*1e9 + (double)(end.tv_nsec);
    Mat estimated;
	 while (1)
	 {
			if (mousev.size () > 1)
			{
				 // First predict, to update the internal statePre variable
				 Mat prediction = KF.predict ();
				 Point predictPt (prediction.at < float >(0),
													prediction.at < float >(1));

            start = end;
	         clock_gettime (CLOCK_REALTIME, &end);
	         startns = endns;
         	endns = (double)(end.tv_sec)*1e9 + (double)(end.tv_nsec);
         	realDts = (endns - startns) * 1e-9;

				 // Get mouse point
				 measurement (0) = mousev[0].x;
				 measurement (1) = mousev[0].y;
             
             if (n % 3 == 0)
             {
				 // The update phase 
				 cout << "--------------------------------------------------------------------------------" << endl << n << endl
				 << "RealDTS : " << realDts << endl
			      << "Measurement : " << measurement.t() << endl
				  << "statePre: " << KF.statePre.t() << endl;

				 estimated = KF.correct (measurement);

				  cout << "statePost: " << KF.statePost.t() << endl
				  << "H: " << KF.measurementMatrix << endl 
				  << "errorCovPost: " << KF.errorCovPost <<endl 
				  << "Gain: " << KF.gain <<endl << endl;
				 }

            if (n >= 3)
            {
				 Point statePt (estimated.at < float >(0), estimated.at < float >(1));
				 Point measPt (measurement (0), measurement (1));
				 // plot points
				 imshow ("mouse kalman", img);
				 img = Scalar::all (0);

				 mousev.push_back (measPt);
				 kalmanv.push_back (statePt);
				 drawCross (statePt, Scalar (255, 255, 255), 5);
				 drawCross (measPt, Scalar (0, 0, 255), 5);

				 for (uint i = 0; i < mousev.size () - 1; i++)
						line (img, mousev[i], mousev[i + 1], Scalar (255, 255, 0), 1);

				 for (uint i = 0; i < kalmanv.size () - 1; i++)
						line (img, kalmanv[i], kalmanv[i + 1], Scalar (0, 155, 255), 1);
				}
			}
			val = waitKey ((int)dtm);
			if (val == 27)
			    break;
			n++;
	 }

	 return 0;
}
