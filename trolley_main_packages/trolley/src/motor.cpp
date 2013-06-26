/*
 * motor.cpp
 *
 *  Created on: Mar 12, 2013
 *      Author: tomaskolo
 */

#include "motor.h"
#include "utilities.h"
#include <geometry_msgs/Twist.h>


Motor::Motor(const std::string & msgType, const std::string & positionMsg)
	: _actualLinearSpeed(0.0), _actualAngularSpeed(0.0), _direction(Direction::NONE)
{
	_motorPublisher = ros::NodeHandle().advertise<geometry_msgs::Twist>(msgType, 1000);
	_positionSubscriber = ros::NodeHandle().subscribe(positionMsg, 10, &Motor::PositionCallback, this);
	_accelerationTimer = ros::NodeHandle().createTimer(ros::Duration(0.05), &Motor::AccelerationCallback, this);
	_accelerationTimer.stop();
}

void Motor::AccelerationCallback(const ros::TimerEvent & event)
{
	boost::mutex::scoped_lock l(_AccelerationMutex);

	if (_maxLinearAcceleration.HasValue()) {

		double step = _maxLinearAcceleration / 20.0;

		if (step < (fabs(_requiredLinearSpeed - _actualLinearSpeed))) {

			if (_requiredLinearSpeed > _actualLinearSpeed) {
				_actualLinearSpeed += step;
			}
			else {
				_actualLinearSpeed -= step;
			}
		}
		else {
			_actualLinearSpeed = _requiredLinearSpeed;
		}
	}
	else {
		_actualLinearSpeed = _requiredLinearSpeed;
	}


	if (_maxAngularAcceleration.HasValue()) {

		double step = _maxAngularAcceleration / 20.0;

		if (step < (fabs(_requiredAngularSpeed - _actualAngularSpeed))) {

			if (_requiredAngularSpeed > _actualAngularSpeed) {
				_actualAngularSpeed += step;
			}
			else {
				_actualAngularSpeed -= step;
			}
		}
		else {
			_actualAngularSpeed = _requiredAngularSpeed;
		}
	}
	else {

		_actualAngularSpeed = _requiredAngularSpeed;
	}

	geometry_msgs::Twist msgTwist;
	msgTwist.linear.x = _actualLinearSpeed;
	msgTwist.angular.z = _actualAngularSpeed;
	_motorPublisher.publish(msgTwist);
}

void Motor::Move(double linearSpeed, double angularSpeed)
{
	_AccelerationMutex.lock();

	_requiredLinearSpeed = linearSpeed;
	_requiredAngularSpeed = angularSpeed;

	_AccelerationMutex.unlock();

	_accelerationTimer.start();
}

void Motor::MaxLinearAcceleration(double maxLinearAcceleration)
{
	boost::mutex::scoped_lock l(_AccelerationMutex);

	_maxLinearAcceleration = maxLinearAcceleration;
}

void Motor::MaxAngularAcceleration(double maxAngularAcceleration)
{
	boost::mutex::scoped_lock l(_AccelerationMutex);

	_maxAngularAcceleration = maxAngularAcceleration;
}

void Motor::UnsetMaxLinearAcceleration()
{
	boost::mutex::scoped_lock l(_AccelerationMutex);

	_maxLinearAcceleration.SetNull();
}

void Motor::Stop()
{
	Move(0.0,0.0);
}

void Motor::RotateToTheRight(double speed)
{
	Move(0.0, -speed);
}

void Motor::RotateToTheLeft(double speed)
{
	Move(0.0, speed);
}

void Motor::MoveUp(double speed)
{
	Move(speed, 0.0);
}

void Motor::MoveDown(double speed)
{
	Move(-speed, 0.0);
}


void Motor::PositionCallback(const nav_msgs::Odometry::ConstPtr& ticks)
{
	boost::mutex::scoped_lock l(_OdometryMutex);

	_actualAngle = (ticks->pose.pose.orientation.z + 1) * 180;


	if (_requiredAngle.IsNull()) {
		return;
	}

	double distance = Utilities::distance_between_two_points_in_degrees(_actualAngle, _requiredAngle);
	if (distance < 10.0) {
		Stop();
		return;
	}

	Direction::Type nextDirection = Utilities::shortest_direction(_actualAngle, _requiredAngle);
	if (nextDirection == Direction::LEFT && _direction == Direction::RIGHT) {
		_requiredAngle.SetNull();
		_direction = Direction::NONE;
	}
	else if (nextDirection == Direction::RIGHT && _direction == Direction::LEFT) {
		_requiredAngle.SetNull();
		_direction = Direction::NONE;
	}
	else if (nextDirection == Direction::LEFT) {
		_direction = Direction::LEFT;
		RotateToTheLeft(0.6);
	}
	else if (nextDirection == Direction::RIGHT) {
		_direction = Direction::RIGHT;
		RotateToTheRight(0.6);
	}
	else if (nextDirection == Direction::NONE) {
		_requiredAngle.SetNull();
		_direction = Direction::NONE;
	}
}

bool Motor::IsSetAngle()
{
	return _requiredAngle.HasValue();
}

void Motor::SetAngle(double degrees)
{
	boost::mutex::scoped_lock l(_OdometryMutex);

	degrees = Utilities::degrees_normalize(degrees);

	if (_positionSubscriber.getNumPublishers() > 0) {

		_requiredAngle = degrees;
	}
	else {
		Stop();
	}
}

Nullable<double> Motor::ActualAngle()
{
	boost::mutex::scoped_lock l(_OdometryMutex);

	return _actualAngle;
}

void Motor::Rotate(double degrees)
{
	boost::mutex::scoped_lock l(_OdometryMutex);

	if (degrees < 15 && degrees > -15) {
		Stop();
	}
	else if(_actualAngle.HasValue()) {

		degrees = Utilities::degrees_normalize(_actualAngle + degrees);

		if (_positionSubscriber.getNumPublishers() > 0) {

			_requiredAngle = degrees;
		}
		else {
			Stop();
		}
	}
}



