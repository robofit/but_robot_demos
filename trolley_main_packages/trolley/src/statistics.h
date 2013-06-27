/*
 * arithmeticMean.h
 *
 *  Created on: Mar 31, 2013
 *      Author: tomaskolo
 */

#ifndef ARITHMETICMEAN_H_
#define ARITHMETICMEAN_H_

#include <exception>
#include <math.h>
#include <boost/static_assert.hpp>

template <class T>
struct IsPointer {
	enum { value = 1 };
};

template <class T>
struct IsPointer<T*> {
	enum { value = 0 };
};

template <class type, unsigned int windowSize>
class Statistics {
public:
	BOOST_STATIC_ASSERT_MSG(IsPointer<type>::value, "Type can not be pointer");

	Statistics(type initValue = 0) : _index(0) {

		for (unsigned int i = 0; i < windowSize; ++i) {

			_window[i] = initValue;
		}
	}

	void addValue(type value) {

		_window[_index] = value;
		++_index;
		_index = _index % windowSize;
	}

	double ArithmeticMean() const {

		double result = 0.0;

		for (unsigned int i = 0; i < windowSize; ++i) {

			result += _window[i];
		}

		return result/windowSize;
	}

	double Variance() const {

		double mean = ArithmeticMean();
		double result = 0.0;

		for (unsigned int i = 0; i < windowSize; ++i) {

			result += (_window[i] - mean) * (_window[i] - mean);
		}

		return sqrt(result/windowSize);
	}

private:
	unsigned int _index;

	type _window[windowSize];

};

template <class type>
class Statistics<type, 0> {
public:

	Statistics() {

		//throw std::runtime_error("Window size must be nonzero");
	}
};

#endif /* ARITHMETICMEAN_H_ */
