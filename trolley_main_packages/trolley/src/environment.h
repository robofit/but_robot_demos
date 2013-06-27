/*
 * environment.h
 *
 *  Created on: Apr 16, 2013
 *      Author: tomaskolo
 */

#ifndef ENVIRONMENT_H_
#define ENVIRONMENT_H_

#include "PID.h"
#include "motor.h"
#include "control.h"
#include "audio.h"
#include "strategy.h"

class control;

/**
 * Prostredi.
 *
 * Nastavni parametru z konfiguracniho souboru.
 */
class Environment {
	friend class Control;
public:
	/**
	 * Konstruktor.
	 */
	Environment();

	/**
	 * Nacteni prostredi ze souboru.
	 */
	bool Load(const char * fileName);

private:

	/// PID regulator pro linearni smer.
	PID _linearPID;

	/// PID regulator pro uhlovy smer.
	PID _angularPID;

	/// Typ strategie pri ztrate sledovane osoby.
	Strategy::Type _strategy;

	/// Motor.
	Motor _motor;

	/// Audio.
	Audio _audio;

	/// Cas.
	ros::Time _lastTime;

};

#endif /* ENVIRONMENT_H_ */
