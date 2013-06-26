/*
 * utilities.cpp
 *
 *  Created on: Apr 3, 2013
 *      Author: tomaskolo
 */

#include "utilities.h"
#include <math.h>
#include <iostream>

double Utilities::distance_between_two_points_in_degrees(double first, double second)
{
	double difference = fabs(first - second);
	if (difference > 180.0) {
		return 360.0 - difference;
	}
	else {
		return difference;
	}
}

double Utilities::degrees_normalize(double value)
{
	return fmod(value, 360.0);
}

Direction::Type Utilities::shortest_direction(double actual, double required)
{
	if (compare_two_doubles(actual, required)) {
		return Direction::NONE;
	}
	else if (actual > required && (actual - required) <= 180) {
		return Direction::RIGHT;
	}
	else if (actual + (360.0 - required) <= 180.0) {
		return Direction::RIGHT;
	}
	else {
		return Direction::LEFT;
	}
}

bool Utilities::compare_two_doubles(double first, double second, double epsilon)
{
	return fabs(first - second) < epsilon;
}
