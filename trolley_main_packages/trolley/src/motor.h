/*
 * motor.h
 *
 *  Created on: Mar 12, 2013
 *      Author: tomaskolo
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "hysteresis.h"
#include "nullable.h"
#include "PID.h"
#include "direction.h"
#include <ros/ros.h>
#include <string>
#include <XnCppWrapper.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread/mutex.hpp>

/**
 *  Rizeni pohybu robota.
 */
class Motor {
public:
	/**
	 * Konstruktor.
	 */
	Motor(const std::string & msgType = "cmd_vel", const std::string & positionMsg = "odom");

	//void Position(Nullable<XnPoint3D> position);

	/**
	 * Rychlost pohybu v linearni a uhlovym smeru.
	 */
	void Move(double linearSpeed, double angularSpeed);

	/**
	 * Pohyb nahoru.
	 */
	void MoveUp(double speed);

	/**
	 * Pohyb dolu.
	 */
	void MoveDown(double speed);

	/**
	 * Otocka.
	 *
	 * @param degrees Uhel na ktery se ma robot natocit.
	 */
	void SetAngle(double degrees);

	/**
	 * Vrati aktualni uhel robota.
	 */
	Nullable<double> ActualAngle();

	/**
	 * Otocka o definovany stupen.
	 */
	void Rotate(double degrees);

	/**
	 * Otocka vpravo.
	 */
	void RotateToTheRight(double speed);

	/**
	 * Otocka vlevo.
	 */
	void RotateToTheLeft(double speed);

	/**
	 * Zastaveni.
	 */
	void Stop();

	/**
	 * Nastaveni maximalni linearni akcelerace.
	 */
	void MaxLinearAcceleration(double maxLinearAcceleration);

	/**
	 * Vrati maximalni nastavenou linearni akceleraci.
	 */
	double MaxLinearAcceleration();

	/**
	 * Odnastaveni akcelerace.
	 */
	void UnsetMaxLinearAcceleration();

	/**
	 * Odnastveni akcelerace.
	 */
	void MaxAngularAcceleration(double maxAngularAcceleration);

	/**
	 * Nastaveni maximalni uhlovou akcelerace.
	 */
	double MaxAngularAcceleration();

	/**
	 * Vrati maximalni nastavenou uhlovou akceleraci.
	 */
	void UnsetMaxAngularAcceleration();

	/**
	 * Vrati true pokud se podvozek otaci na definovany uhel.
	 */
	bool IsSetAngle();

private:

	/// Zakazani copy konstruktoru.
	Motor(const Motor & other);

	/// Callback pro odometrii
	void PositionCallback(const nav_msgs::Odometry::ConstPtr& ticks);

	/// Callback pro akceleraci.
	void AccelerationCallback(const ros::TimerEvent & event);

	///
	Nullable<double> _requiredLinearSpeed;

	///
	double _actualLinearSpeed;

	///
	Nullable<double> _requiredAngularSpeed;

	///
	double _actualAngularSpeed;

	///
	Nullable<double> _maxLinearAcceleration;

	///
	Nullable<double> _maxAngularAcceleration;

	///
	Direction::Type _direction;

	///
	Nullable<double> _requiredAngle;

	///
	Nullable<double> _actualAngle;

	///
	ros::Publisher _motorPublisher;

	///
	ros::Subscriber _positionSubscriber;

	///
	ros::Timer _accelerationTimer;

	///
	boost::mutex _AccelerationMutex;

	///
	boost::mutex _OdometryMutex;

	//Hysteresis _angularLeftHysteresis;
	//Hysteresis _angularRightHysteresis;
	//Hysteresis _linearLeftHysteresis;
	//Hysteresis _linearRightHysteresis;
};

#endif /* MOTOR_H_ */

