/*
 * PID.cpp
 *
 *  Created on: Mar 10, 2013
 *      Author: tomaskolo
 */

#include "PID.h"
#include <stdlib.h>
#include <iostream>
#include <math.h>


PID::PID(double MAX, double MIN, double Kp, double Ki, double Kd, double epsilon)
	: _MAX(MAX), _MIN(MIN), _Kp(Kp), _Ki(Ki), _Kd(Kd), _epsilon(epsilon), pre_error(0.0), integral(0.0), firstTime(true)
{
}

double PID::Calculate(double required, double actual, double dt)
{
	//Caculate P,I,D
	double error = required - actual;

	if (firstTime) {
	    //Update error
	    pre_error = error;

	    firstTime = false;
	}

	//In case of error too small then stop intergration
	if(fabs(error) > _epsilon) {

		integral = integral + error * dt;
	}

	double derivative = (error - pre_error)/dt;

	//double Td = 0.52380;
	//double N = 10;
	//_derivative = (Td/(Td + N * dt)* _derivative) - ((_Kp*Td*N) / (Td + N * dt))*(error - pre_error);

	double output = _Kp*error + _Ki*integral + _Kd*derivative;

	//Saturation Filter
	if(output > _MAX) {
		output = _MAX;
	}
	else if(output < _MIN) {
		output = _MIN;
	}

    //Update error
    pre_error = error;

    return output;
}

void PID::Clear()
{
	firstTime = true;
	pre_error = 0.0;
	integral = 0.0;
}
