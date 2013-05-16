/*
 * control.h
 *
 *  Created on: Apr 15, 2013
 *      Author: tomaskolo
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include "nullable.h"
#include "environment.h"
#include <ros/ros.h>
#include <XnCppWrapper.h>

class Environment;

/**
 * Trida se stara o ovladani pohybu robota
 * na zaklade pozice osoby ziskane z OpenNI.
 *
 * Pokud data nejsou k dispozici, probiha detekce
 * na zaklade informaci z mikrofonu.
 */

class Control
{
public:
	/**
	 *  Konstruktor.
	 */
	Control(Environment * enviroment);

	/**
	 *  Pozice osoby.
	 *
	 *  Pozice zistana z NITE.
	 */
	void Position(Nullable<XnPoint3D> position);

private:

	/// Prostredi.
	Environment * _enviroment;

	/// Cas posledniho pruchodu.
	ros::Time _lastTime;
};

#endif /* CONTROL_H_ */
