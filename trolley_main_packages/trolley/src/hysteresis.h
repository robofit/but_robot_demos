/*
 * hysteresis.h
 *
 *  Created on: Mar 4, 2013
 *      Author: tomaskolo
 */

#ifndef HYSTERESIS_H_
#define HYSTERESIS_H_

/**
 * Hystereze.
 *
 * Usetruje zakmity.
 */
class Hysteresis {
public:
	/**
	 * Konstruktor.
	 */
	Hysteresis(double leftThreshold, double rightThreshold);

	/**
	 * Stav hystereze.
	 */
	bool GetState(double value);

	/**
	 * Levy prah.
	 */
	void SetLeftThreshold(double value);

	/**
	 * Pravy prah.
	 */
	void SetRightThreshold(double value);


	/**
	 * Levy a pravy prah.
	 */
	void SetThreshold(double left, double right);

private:


	double _actualThreshold;

	/// Aktualni stav
	bool _state;

	double _leftThreshold;

	double _rightThreshold;
};

#endif /* HYSTERESIS_H_ */
