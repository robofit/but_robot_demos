/*
 * utilities.h
 *
 *  Created on: Apr 3, 2013
 *      Author: tomaskolo
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_

#include "direction.h"

namespace Utilities
{
	/**
	 * Vzdalenost mezi dvema body.
	 */
	double distance_between_two_points_in_degrees(double first, double second);

	/**
	 * Prevede uhel do intervalu <0, 360).
	 */
	double degrees_normalize(double value);

	/**
	 * Nejkratsi smer k danemu uhlu.
	 */
	Direction::Type shortest_direction(double actual, double required);

	/**
	 * Porovnani dvou doubles.
	 */
	bool compare_two_doubles(double first, double second, double epsilon = 1E-8);
}

#endif /* UTILITIES_H_ */
