//
// Created by dayan3847 on 15/10/23.
//

#ifndef LIB_ARTURO_H
#define LIB_ARTURO_H

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <vector>
#include <cstdio>
//#include <Circle.h>

#define SLIDE_MAX 1000                    //!< El valor maximo de pasos del control de barra deslizante del GUI.

#define IM_WIDTH 640                        //!< El ancho de la imagen.
#define IM_HEIGHT 480                        //!< Lo alto de la imagen.


#define  a_COMPONENT  30.677        //!< El valor del componente a del modelo de color.
#define  b_COMPONENT 58.212            //!< El valor del componente b del modelo de color.
#define I_FACT (1. / 255.) //!< Factor de escalamiento de la imagen de entrada.

namespace arturo
{

/*!
\fn void printMat(cv::Mat &M, const char *name = NULL, bool transp=false)
\brief Esta funcion imprime en stdout como texto el contenido de una matriz
       almacenado en un objeto CV::Mat.

\param M La matriz a ser impresa
\param name El nombre de la variable o identificador asociado a la matriz.
\param transp Indica si se imprime la matriz de manera convencional o
       transpuesta.

 Esta funcion imprime en stdout como texto el contenido de una matriz almacenado en un objeto CV::Mat.
 El formato de impresión sigue la sintaxis aceptada por Matlab/octave y tiene
 una opcion para imprimir en forma transpuesta (añadiendo un apostrofe al final
 de la definicion), lo cual es útil cuando se imprimen vectores columna en forma
 de renglones.

*/
	template<typename X>
	void printMat(cv::Mat& M, const char* name = NULL, bool transp = false)
	{
		int i, j;

		if (name)
			std::cout << name << " = [";
		else
			std::cout << name << "[";

		if (transp)
		{
			for (i = 0; i < M.cols; ++i)
			{
				for (j = 0; j < M.rows - 1; ++j)
					std::cout << M.at<X>(i, j) << ", ";
				std::cout << M.at<X>(i, j) << std::endl;
			}
			std::cout << "]'" << std::endl;
		}
		else
		{
			for (i = 0; i < M.rows; ++i)
			{
				for (j = 0; j < M.cols - 1; ++j)
					std::cout << M.at<X>(i, j) << ", ";
				std::cout << M.at<X>(i, j) << std::endl;
			}
			std::cout << "]" << std::endl;
		}
	}

/*!
\fn int MeaniCov(cv::Mat &image, cv::Mat &Mask, cv::Mat &mean, cv::Mat &cov)

\brief Esta funcion calcula la media y la inversa de la matriz de covarianza de
cada uno de los elementos de una matriz que representa una imagen a color.
\param image La imagen a la que se le va a calcular la media y la matriz de
             covarianza.
\param Mask Una matriz que binaria que se usa para determinar sobre que
            elementos de imagen se va a realizar el cómputo.
\param mean Una matriz de 3 renglones y 1 columna, en donde se regresa el vector
            promedio de los pixeles de la imagen.
\param mean Una matriz de 3 renglones y 3 columnas, en donde se regresa la
            matriz de covarianza de los pixeles de la imagen.
\return El número de elementos procesados. Regresa el valor -1 si el número de
        elementos a procesar es menor a 2.

Esta funcion calcula la media y la inversa de la matriz de covarianza de cada
uno de los elementos de una matriz que representa una imagen a color. Como
parámetro se recibe una matriz de mascara, del mismo tamaño que la imagen de
entrada, con lo cual podemos controlar de manera fina que elementos de la matriz
de entrada se deben procesar. La funcion regresa el número de elementos
procesados (que es el número de elementos de la matriz de mascara diferentes a
0); si ese número es menor a 1, la funcion regresa el valor -1 para indicar un
fallo, y como no se puede calcular ni el vector promedio ni la matriz de
covarianza los valores de los parámteros por referencia mean y cov son
indeterminados. Si el número de elementos procesado es igual a 1, el parametro
por referencia cov es valido y contiene el valor del elemento procesado, pero el
parametro por referencia cov es indeterminado, y la funcion indica fallo
regresando el valor -1.
*/

	int MeaniCov(cv::Mat& image, cv::Mat& Mask, cv::Mat& mean, cv::Mat& icov)
	{
		float m[2], pm[2], Cv[3], iCont, iFact;
		int cont;
		mean = cv::Mat::zeros(2, 1, CV_32F);
		icov = cv::Mat::zeros(2, 2, CV_32F);
		cv::Mat_<cv::Vec3f>::iterator it, itEnd;
		cv::Mat_<uchar>::iterator itM;

		it = image.begin<cv::Vec3f>();
		itM = Mask.begin<uchar>();
		itEnd = image.end<cv::Vec3f>();
		m[0] = m[1] = 0;
		memset(m, 0, 2 * sizeof(float));
		for (cont = 0; it != itEnd; ++it, ++itM)
		{
			if ((*itM))
			{
				m[0] += (*it)[1];
				m[1] += (*it)[2];
				cont++;
			}
		}

		if (!cont)
			return -1;
		m[0] /= cont;
		m[1] /= cont;
		mean = cv::Mat(2, 1, CV_32F, m).clone();

		if (cont < 1)
		{
			icov.at<float>(0, 0) = icov.at<float>(1, 1) =
			icov.at<float>(2, 2) = 1.;
			return -1;
		}
		it = image.begin<cv::Vec3f>();
		itM = Mask.begin<uchar>();
		memset(Cv, 0, 3 * sizeof(float));
		for (; it != itEnd; ++it, ++itM)
		{
			if ((*itM))
			{
				pm[0] = (*it)[1] - m[0];
				pm[1] = (*it)[2] - m[1];
				Cv[0] += pm[0] * pm[0];
				Cv[1] += pm[1] * pm[1];
				Cv[2] += pm[0] * pm[1];
			}
		}
		cont--;
		iCont = 1. / cont;
		Cv[0] *= iCont;
		Cv[1] *= iCont;
		Cv[2] *= iCont;

		iFact = 1. / (Cv[0] * Cv[1] - Cv[2] * Cv[2]);
		icov.at<float>(0, 0) = Cv[1] * iFact;
		icov.at<float>(1, 1) = Cv[0] * iFact;
		icov.at<float>(1, 0) = icov.at<float>(0, 1) = Cv[2] * iFact;

		return cont;
	}

/*!
\struct barData
\brief Esta estructura almacena el valor inicial y el factor de incremento de
una barra deslizante (un elemento del GUI usada en el programa).
*/
	struct barData
	{
		float fact; //!< Esta variable almacena el factor de incremento utilizado por la barra deslizante.

		float val;    //!< Este valor almacena el valor inicial de la barra deslizante.

		/*!
		   \fn barData(float f, float v)
		   \brief Constructor de la clase; inicializa los attributos fact y val.
		   \param f El valor con el que inicializamos el atributo fact.
		   \param v El valor con el que inicializamos el atributo val.
		*/
		barData(float f, float v)
		{
			fact = f;
			val = v;
		}
	};

/*!
\fn void umLuzChange (int pos, void *data)
\brief Esta función es invocada por el GUI cada vez que el usuario interactua
con la barra deslizante asociada a un umbral de intensidad de luz.
\param pos Aquí se pasa la nueva posición de la barra
\param data Un apuntador a los datos asociados a la barra.

Esta función es invocada por el GUI cada vez que el usuario interactua con la
barra deslizante asociada a un umbral de intensidad de luz.
Cuando el usuario modifica la posición de la barra, se invoca esta fución, al
ser invocada se le pasa como parámetro la nueva posicion (con el parámetro pos),
y aun apuntador generico a datos que el usuario puede utilizar para modificar el
funcionamiento del programa (en nuestro caso una estructura del tipo barData).
*/
	void umLuzChange(int pos, void* data)
	{
		barData* umbral = (barData*)data;

		umbral->val = pos * umbral->fact;
	}

/*!
\fn void umDistChange (int pos, void *data)
\brief Esta función es invocada por el GUI cada vez que el usuario interactua
con la barra deslizante asociada a un umbral de distancia.
\param pos Aquí se pasa la nueva posición de la barra
\param data Un apuntador a los datos asociados a la barra.

Esta función es invocada por el GUI cada vez que el usuario interactua con la
barra deslizante asociada a un umbral de distancia
Cuando el usuario modifica la posición de la barra, se invoca esta fución, al
ser invocada se le pasa como parámetro la nueva posicion (con el parámetro pos),
y aun apuntador generico a datos que el usuario puede utilizar para modificar el
funcionamiento del programa (en nuestro caso una estructura del tipo barData).
*/
	void umDistChange(int pos, void* data)
	{
		barData* umbral = (barData*)data;

		umbral->val = pow(pos * umbral->fact, 2);
		std::cout << "Umbral Dist :" << umbral->val << std::endl;
	}

/*!
\fn void Umbraliza(cv::Mat &Im, cv::Mat &Mask, cv::Mat &M, cv::Mat &iCov, float umD, float umL)
\brief Esta funcion umbraliza la imagen a color en base a la distancia de Mahalanobis a un modelo de color.
\param Im Un objeto del tipo CV::Mat que reoresenta una imagen a color con tres
          canales.
\param Mask Una referencia a un objeto del tipo CV::Mat en donde se regresa la
            máscara que indica que elementos de Im están arriba del umbral.
\param M Un objeto del tipo CV::Mat que contiene el vector promedio del modelo
       de color con el cual se compara.
\param iCov Un objeto del tipo CV::Mat que contiene el inverso de la matriz de
            convarianza de modelo de color con el cual se compara.
\param umD El valor del umbral de distancia entre cada elemento procesado y el
           modelo de color.
\param umL El valor del umbral de intensidad luminosa.

Esta funcion umbraliza la imagen a color en base a la distancia de Mahalanobis
a un modelo de color. El modelo de color esta determinado por el vector M, y la
matriz iCov, que almacenan el color promedio y la inversa de la matrix de
covarianza del modelo de color. El proceso de umbralización ocurre en dos
niveles: primero se descarta aquellos elementos cuya intensidad luminosa es
inferior al vlor umL, esto con el fin de eliminar pixeles obscuros, y segundo se
eliminan aquellos pixeles cuya distancia de Mahalanobis al modelo de color es
mayor el umD.

La imagen umbralizada se regresa en la matriz Mask.
*/
	void Umbraliza(cv::Mat& Im, cv::Mat& Mask, cv::Mat& M, cv::Mat& iCov, float umD,
		float umL)
	{

		cv::Mat_<cv::Vec3f>::iterator it, itEnd;
		cv::Mat_<uchar>::iterator itM;
		float ligth, maha, ma, mb, va, vb, q, r, s;
		double meanMaha = 0;
		int cont = 0;

		it = Im.begin<cv::Vec3f>();
		itEnd = Im.end<cv::Vec3f>();
		itM = Mask.begin<uchar>();
		ma = M.at<float>(0, 0);
		mb = M.at<float>(0, 1);
		q = iCov.at<float>(0, 0);
		r = iCov.at<float>(1, 0);
		s = iCov.at<float>(1, 1);

		for (; it != itEnd; ++it, ++itM)
		{
			ligth = (*it)[0];                    //Solo analizamos pixeles lo suf. brillantes.
			if (ligth > umL)
			{
				// Restamos el vector promedio a cada pixel.
				va = (*it)[1] - ma;
				vb = (*it)[2] - mb;

				//Calculamos la distancia de mahalanobis del pixel al modelo
				// [va,vb]*iCov*[va;vb]
				maha = vb * (s * vb + r * va) + va * (r * vb + q * va);
				meanMaha += maha;
				cont++;
				if (maha < umD)
					*itM = 255;
				else
					*itM = 0;
			}
			else
				*itM = 0;
		}
#ifdef __VERBOSE__
		if (cont)
			std::cout << "Mean Mahalanobis Distance: " << meanMaha << "/" << cont <<
				 " : " << meanMaha / cont << std::endl;
#endif
	}

	void convertLab(cv::Mat const& frame, cv::Mat& labFrame)
	{
		cv::Mat tempFrame;
		frame.convertTo(tempFrame, CV_32FC3);
		//Es necesario normalizar la image BGR al intervalo [0,1] antes de convertir a espacio CIE Lab; en este caso iFact = 1./255
		tempFrame *= I_FACT;
		cvtColor(tempFrame, labFrame, cv::COLOR_BGR2Lab);
	}
}

#endif //LIB_ARTURO_H
