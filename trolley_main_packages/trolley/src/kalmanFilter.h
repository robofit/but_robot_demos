/*
 * kalmanFilter.h
 *
 *  Created on: Feb 19, 2013
 *      Author: tomaskolo
 */

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "math.h"

/**
 * Kalmanuv filtr.
 *
 * Objekt je definovany polohou a rychlosti (zrychleni je mozny pridat).
 */

namespace Math
{
	class KalmanFilter
	{
	public:
		/**
		 * Konstruktor.
		 */
		KalmanFilter(double dt, double processNoisePSD, double measurementNoiseVariance);

		/**
		 * Predikce.
		 */
		void Predict();

		/**
		 * Korekce.
		 */
		void Correct(const Matrix<2,1> & Z);

		/**
		 * Odhad dalsho stavu bez korekce.
		 */
		void Correct();

		/**
		 * Stavovy vektor.
		 */
		void SetX(const Matrix<4,1> & X);

		/**
		 * Stavovy vektor.
		 */
		const Matrix<4,1> & GetX() const;

    private:

		/// Stavovy vektor.
		Matrix<4,1> X;

		/// atd
		Matrix<4,1> X0;

		///
		Matrix<4,4> F;

		///
		Matrix<4,4> Q;

		///
		Matrix<2,4> H;

		///
		Matrix<2,2> R;

		///
		Matrix<4,4> P;

		///
		Matrix<4,4> P0;
	};
}


#endif /* KALMANFILTER_H_ */
