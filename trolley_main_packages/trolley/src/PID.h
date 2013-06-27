/*
 * PID.h
 *
 *  Created on: Mar 10, 2013
 *      Author: tomaskolo
 */

#ifndef PID_H_
#define PID_H_

#include "kalmanFilter.h"

/**
 * PSD regulator.
 */
class PID {
public:
	/**
	 * Konstruktor.
	 */
	PID(double MAX, double MIN, double Kp, double Ki, double Kd, double epsilon = 0.0);

	/**
	 * Vypocet hodnoty.
	 */
	double Calculate(double required, double actual, double dt);

	/**
	 * Reset PID.
	 */
	void Clear();

private:

	///< Omezeni vychylky.
	double _MAX;

	///< Omezeni vychylky.
	double _MIN;

	///< Proporcionalni slozka.
	double _Kp;

	///< Sumacni slozka.
	double _Ki;

	///< Derivacni slozka.
	double _Kd;

	///<
	double _epsilon;

	double pre_error;

	///< Sumacni clen.
	double integral;

	///< Prvni pruchod.
	bool firstTime;

};

#endif /* PID_H_ */
