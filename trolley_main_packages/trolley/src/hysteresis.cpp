/*
 * hysteresis.cpp
 *
 *  Created on: Mar 4, 2013
 *      Author: tomaskolo
 */
#include <stdexcept>
#include "hysteresis.h"

Hysteresis::Hysteresis(double leftThreshold, double rightThreshold)
	: _actualThreshold(rightThreshold), _state(false), _leftThreshold(leftThreshold), _rightThreshold(rightThreshold)
{
	if (leftThreshold >= rightThreshold) {
		// Vyhodi vyjimku. Leva strana musi byt mensi nez prava.
		throw std::runtime_error("Bad threshold.");

	}
}

bool Hysteresis::GetState(double value)
{
	if (value > _actualThreshold) {
		// Aktualni hodnota prerostla pres pravy prah.
		_actualThreshold = _leftThreshold;
		// Zmen stav na true.
		_state = true;
	}
	else if (value < _actualThreshold) {
		// Aktualni hodnota prerostla pres levy prah.
		_actualThreshold = _rightThreshold;
		// Zmen stava na false.
		_state = false;
	}

	// Vrati stav hystereze.
	return _state;
}

void Hysteresis::SetLeftThreshold(double value)
{
	_leftThreshold = value;
}

void Hysteresis::SetRightThreshold(double value)
{
	_rightThreshold = value;
}

void Hysteresis::SetThreshold(double left, double right)
{
	_leftThreshold = left;
	_rightThreshold = right;
}
