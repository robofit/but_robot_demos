/*
 * direction.h
 *
 *  Created on: Apr 3, 2013
 *      Author: tomaskolo
 */

#ifndef DIRECTION_H_
#define DIRECTION_H_

/**
 *  Vyctovy typ pro smer robota.
 */
namespace Direction
{
	enum Type
	{
		NONE,
		LEFT,
		RIGHT,
		UP,
		DOWN,
		CUSTOMER
	};
}

#endif /* DIRECTION_H_ */
