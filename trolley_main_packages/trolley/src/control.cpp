
/*
 * control.cpp
 *
 *  Created on: Apr 15, 2013
 *      Author: tomaskolo
 */

#include "control.h"
#include "strategy.h"
#include <math.h>



Control::Control(Environment * enviroment)
	: _enviroment(enviroment), _lastTime(ros::Time::now())
{
}

void Control::Position(Nullable<XnPoint3D> position)
{

/*


*/




	if (position.HasValue()) {

		double Z= position.GetValue().Z;
		double X = position.GetValue().X;

		double timeDiff = (ros::Time::now() - _lastTime).toNSec() * 1.0e-9;

		double newLinearSpeed = _enviroment->_linearPID.Calculate(1200, Z, timeDiff);
		double newAngularSpeed = _enviroment->_angularPID.Calculate(0.0, X, timeDiff);

		_enviroment->_motor.Move(-newLinearSpeed, newAngularSpeed * (fabs(newLinearSpeed) * 10 + 1));
		_enviroment->_motor.UnsetMaxLinearAcceleration();
	}
	else {

		_enviroment->_motor.MaxLinearAcceleration(0.5);
		_enviroment->_linearPID.Clear();
		_enviroment->_angularPID.Clear();

		switch(_enviroment->_strategy)
		{

		case Strategy::FIRST:
		{
			Nullable<double> angle = _enviroment->_audio.Angle();

			if (angle.HasValue()) {

				_enviroment->_motor.MaxLinearAcceleration(0.5);
				_enviroment->_motor.Rotate(angle.GetValue());

			}
			else if (!_enviroment->_motor.IsSetAngle()) {

				_enviroment->_motor.Stop();
			}

			break;
		}
		case Strategy::SECOND:

			_enviroment->_motor.Stop();

			// Doimplenetovat provedeni jedny otocky pomoci metody "SetAngle".

			break;

		case Strategy::THIRD:

			_enviroment->_motor.Stop();

			break;
		}




	}

	_lastTime = ros::Time::now();


}

