/*
 * strategy.h
 *
 *  Created on: May 14, 2013
 *      Author: tomaskolo
 */

#ifndef STRATEGY_H_
#define STRATEGY_H_

/**
 * Typ strategie.
 */
namespace Strategy
{
	enum Type
	{
		FIRST, ///< Naslouchani z mikrofonu.
		SECOND, ///< Jedna otocka.
		THIRD, ///< Vyckavani.
	};
}


#endif /* STRATEGY_H_ */
