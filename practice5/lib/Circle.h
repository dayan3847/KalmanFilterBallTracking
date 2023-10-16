//
// Created by dayan3847 on 15/10/23.
//

#ifndef CIRCLE_H
#define CIRCLE_H

#include "opencv2/opencv.hpp"
#include <vector>

using namespace cv;

namespace arturo
{

	/*!
    \file
    \brief The circle class header file.

    This header file contains the declaration of the struct Circle. The struct Circle allows us yo instantiate object capable of representing a 2D circle, whit methods to define the circle from a set of points lying on the circle locus.
*/

/*!
    A shorthand for a coordinate with three elements. In this program the two first elements store the coordinates of a point in a 2D space, and the third element keeps the coordinate class (0==outlier, 1==inlier).
*/
	typedef Point3_<short> Point3s;

	/*!
		\struct Circle
		\brief This structure contains attributes that parametrize circle in the plane, and contain methods to define it uniquely from 3 points, and to perform a robust fit from many points (\f$ >3\f$) using the RANdom SAmple Consensus (RANSAC) method.
	*/
	struct Circle
	{
		/*!
			\var float h, k, r
			\brief Stores the parameters of a circle with radius \f$r\f$ and centered at coordinates \f$(h,\,k)\f$
		*/
		float h, k, r;
		/*!
			\var bool undefined This variable indicates that the circle has not been defined.
		*/
		bool undefined;

		/*!
			\func Circle ()
			\brief Object constructor.
		*/
		Circle()
		{
			h = k = r = 0.;
		}

		/*!
			\func Circle(float x, float y, float radius);
			\brief Object constructor (with paramaters).
			\param x The position of the circle center along the X-axis.
			\param y The position of the circle center along the Y-axis.
			\param r The circle radius.
		*/
		Circle(float x, float y, float radius);

		/*!
			\func Circle(vector <Point3s> &p)
			\brief Object constructor (with paramaters). This constructor is the most general constructor since it will define the circle parameters from a set of 3 or more point's coordinates that lie on the circles loci. If the points are colinear or there are less than 3 points, the circle is not defined; if there are exactly 3 non-colinear points, the circle gets defines uniquely, and if there are more than three points, the circle is fitted to the data using the RANSAC algoritm.
			\param vector <Point3s> &p A vector of coordinates that lie on the circles locus.
		*/
		Circle(std::vector<Point3s>& p);

		/*!
			 \func float fitCircle(std::vector<Point3s> &pts, float w=0.6, float sigma = 1, float p=0.99)
			 \brief This function find the circle parameters from a set of point coordinates. If there are less than three points in the set or there are exactly three and they are colinear, the circle is not defined, and the function returns -1. if there are more than three points, the circle is fitted to the data using the RANSAC algoritm; in this case the function returns the fit error, defined as the the average of the minimum distance between the points use to define the circle, and the circle found. The parameters w, sigma and p are only needed in case there are more than 3 points in the set.
			 \param vector <Point3s> &p A vector of coordinates used to define a circle.
			 \param float w Is the proportion of inliers vs outliers.
			 \param float sigma Is the measurement noise.
			 \param float p The probability that the points used to define the circle is outlier-free.
			 \return This function returns the fit error, 0 is there is a unique fit, and -1 if the circle is not defined.
		 */
		float fitCircle(std::vector<Point3s>& pts, float w = 0.6, float sigma = 1, float p = 0.99);

		/*!
		\func void fitCircle( float x1, float y1, float x2, float y2, float x3, float y3)
		\brief thsi methis defines a circle from exactly three set of coordinates of three points on the plane. If the points are colinear, the circle is not defined.
		\param x1 The position of the first coordinate along the X-axis.
		\param y1 The position of the first coordinate along the Y-axis.
		\param x2 The position of the second coordinate along the X-axis.
		\param y2 The position of the second coordinate along the Y-axis.
		\param x3 The position of the third coordinate along the X-axis.
		\param y3 The position of the third coordinate along the Y-axis.
		*/
		void fitCircle(float x1, float y1, float x2, float y2, float x3, float y3);

		/*!
			\func float fitBestCircle(vector<Point3s> &pts, unsigned int nInliers, unsigned int *idx)
			\brief Find the circle parameters that best fit the point coordinates stored in pts. The function receives an array with a list of those coordinates in pts that are classified as inliners. This method is usually the last step in the RANSAC method.
			\param vector <Point3s> &pts A vector of coordinates; some of them define a circle.
			\param int nInliers the number of coordinate that are classified as inliers in vector pts
			\param int *idx An array that contains the indexes of those elements of pts that are inliers; the length of the array pointed by idx is nInliers.
			\return The error, defined as the average of the minimum distance between the points use to define the circle, and the circle found.
		*/
		float fitBestCircle(std::vector<Point3s>& pts, unsigned int nInliers, unsigned int* idx);

		/*!
			\func void selectAndTestSample(vector <Point3s> &pts, float thr, unsigned int &inliers, unsigned int &outliers)
			\brief This function randomly select a sample of three points from the pts vector, find the circle that fits those the points, and count how many of them are inliers and how many are outliers; A point is an inlier if its distance to the circle is smaller or equal that a threshold (parameter thr), otherwise is deemed to be an outlier. The third component of each coordinate (Point3s contain three elements), store the status of the coordinate: 1 == inliers, 0 == outlier.
			\param vector <Point3s> &pts A vector of coordinates that might lie on a circles locus.
			\param float thr The threshold used to determine if a point is an inlier or an outlier.
			\param unsigned int &inliers used to  returns the number of inliers found.
			\param unsigned int &outliers used to return the number of outliers found.
		*/
		void selectAndTestSample(std::vector<Point3s>& pts, float thr, unsigned int& inliers, unsigned int& outliers);

		/*!
			\func float ransacFit(vector <Point3s> &pts, unsigned int &nInl, float w, float sigma = 1, float p=0.99)
			\brief This function implements the RANSAC algorithm to fit a circle to a set of points.
			\param vector <Point3s> &pts A vector of coordinates used to define a circle. After the function eecution the third component of each element of pts is equal to 0 if that coordinate is an outlier, and 1 otherwise.
			\param unsigned int &nInl Return the number of inliers found.
			\param float w Is the proportion of inliers vs outliers.
			\param float sigma Is the measurement noise.
			\param float p The probability that the points used to define the circle is outlier-free.
			\return The error, defined as the average of the minimum distance between the points use to define the circle, and the circle found.
		*/
		float ransacFit(std::vector<Point3s>& pts, unsigned int& nInl, float w, float sigma = 1, float p = 0.99);

		/*!
			\func float distMin(Point3s &p)
			\brief  Computes and return the minimum distance between the point p and the circle.
			\param Point3s &p The point for which the distance will be computed.
			\return the minimum distance between point p and the circle.
		*/
		float distMin(Point3s& p);
	};

} // arturo

#endif //CIRCLE_H
