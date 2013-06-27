/*
 * nullable.h
 *
 *  Created on: Mar 11, 2013
 *      Author: tomaskolo
 */

#ifndef NULLABLE_H_
#define NULLABLE_H_

#include <stdexcept>

/*
template <class T>
struct IsPointer {
	enum { value = 0 };
};

template <class T>
struct IsPointer<T*> {
	enum { value = 1 };
};
*/

/**
 * Nullable can be used to represent a value type such that
 * the type can be nulled. You can also compare it to nullptr using
 * the == and != operators both of which have been overloaded.
 */
template <class Type>
class Nullable
{
	// Type must not be a pointer.
	// Static_asert....
public:

	/**
	 * Constructor.
	 */
	Nullable()
		: _isNull(true)
	{
	}

	/**
	 * Copy constructor.
	 */
	Nullable(const Type & value)
		: _value(value), _isNull(false)
	{
	}

	/**
	 * Constructor for the representation null value.
	 *
	 * Argument can be only nullptr.
	 */
	Nullable(const Type * /* pointer */)
		: _isNull(true)
	{
	}

	/**
	 * Assignment operator for nullptr.
	 */
	Nullable & operator=(const Type * /* pointer */)
	{
		SetNull();
		return *this;
	}

	/**
	 * Set value.
	 */
	void SetValue(const Type & value)
	{
		_value = value;
		_isNull = false;
	}

	/**
	 * Set object to null.
	 */
	void SetNull(bool isNull = true)
	{
		_isNull = isNull;
	}

	/**
	 * Get value.
	 */
	Type & GetValue()
	{
		if (_isNull) {
			throw std::runtime_error("Object does not have a value.");
		}

		return _value;
	}

	/**
	 * Get value.
	 */
	const Type & GetValue() const
	{
		if (_isNull) {
			// Object does not have a value.
			throw std::runtime_error("Object does not have a value.");
		}

		return _value;
	}

	/**
	 * Cast operator.
	 */
	operator const Type & () const
	{
		if (_isNull) {
			// Object does not have a value.
			throw std::runtime_error("Object does not have a value.");
		}
		return _value;
	}

	/**
	 * Return true in case the object is null.
	 */
	bool IsNull() const
	{
		return _isNull;
	}

	/**
	 * Return true in case the object has a value.
	 */
	bool HasValue() const
	{
		return !_isNull;
	}

	/**
	 * == operator.
	 */
	bool operator==(const void * /* pointer */) const
	{
		return IsNull();
	}

	/**
	 * != operator.
	 */
	bool operator !=(const void * /* pointer */) const
	{
		return HasValue();
	}


private:

	/**
	 * Value of the object.
	 */
	Type _value;

	/**
	 * Flag indicating whether the current object has a value.
	 */
	bool _isNull;
};

#endif /* NULLABLE_H_ */
