/*
 * math.h
 *
 *  Created on: Feb 19, 2013
 *      Author: tomaskolo
 */

#ifndef MATH_H
#define MATH_H

#include <iostream>
#include <stdexcept>
#include "matrixFunctions.h"

namespace Math
{

	/**
	 * Matrix with predefined size.
	 *
	 * This is general template for non-square matrices. Square matrices are implemented as partial
	 * specialization of this template. See the code bellow in this file.
	 */
	template <unsigned int rows, unsigned int columns>
	class Matrix
	{

	public:

		/**
		 * Constructor.
		 */
		Matrix<rows, columns>()
		{
			// Set all values of this matrix to 0.0.
			clear();
		}

		Matrix<rows, columns>(double values[rows][columns])
		{
			for (unsigned int i = 0; i < rows; ++i) {
				for (unsigned int j = 0; j < columns; ++j) {
					_values[i][j] = values[i][j];
				}
			}
		}

		/**
		 * Copy contructor.
		 */
		Matrix<rows, columns>(const Matrix<rows, columns> & other)
		{
			for (unsigned int i = 0; i < rows; ++i) {
				for (unsigned int j = 0; j < columns; ++j) {
					_values[i][j] = other._values[i][j];
				}
			}
		}

		/**
		 * Assignment operator.
		 */
		Matrix<rows, columns> & operator=(const Matrix<rows, columns> & other)
		{
			for (unsigned int i = 0; i < rows; ++i) {
				for (unsigned int j = 0; j < columns; ++j) {
					_values[i][j] = other._values[i][j];
				}
			}

			return (*this);
		}

		/**
		 * Get matrix value from row and column.
		 */
		double operator()(unsigned int row, unsigned int column) const
		{
			return _values[row][column];
		}

		/**
		 * Set matrix value to row and column.
		 */
		double & operator()(unsigned int row, unsigned int column)
		{
			return _values[row][column];
		}

		/**
		 * Get matrix value from row and column.
		 *
		 * This method produces the exception, if indices are out of range.
		 */
		double & at(unsigned int row, unsigned int column)
		{
			if (row < rows && column < columns) {
				return _values[row][column];
			}
			else {
				throw std::out_of_range("Matrix indices out of range.");
			}
		}

		/**
		 * Clear matrix content - set all members to 0
		 */
		void clear()
		{
			for (unsigned int i = 0; i < rows; ++i) {
				for (unsigned int j = 0; j < columns; ++j) {
					_values[i][j] = 0.0;
				}
			}
		}

		/**
		 * Addition.
		 */
		Matrix<rows, columns> & operator+=(const Matrix<rows, columns> & other)
		{
			for (unsigned int row = 0; row < rows; ++row) {
				for (unsigned int column = 0; column < columns; ++column) {
					_values[row][column] += other._values[row][column];
				}
			}

			return *this;
		}

		/**
		 * Addition.
		 */
		Matrix<rows, columns> operator+(const Matrix<rows, columns> & other) const
		{
			return Matrix<rows, columns>(*this) += other;
		}

		/**
		 * Subtraction.
		 */
		Matrix<rows, columns> & operator-=(const Matrix<rows, columns> & other)
		{
			for (unsigned int row = 0; row < rows; ++row) {
				for (unsigned int column = 0; column < columns; ++column) {
					_values[row][column] -= other._values[row][column];
				}
			}

			return *this;
		}

		/**
		 * Subtraction.
		 */
		Matrix<rows, columns> operator-(const Matrix<rows, columns> & other) const
		{
			return Matrix<rows, columns>(*this) -= other;
		}

		/**
		 * Multiplication of the metrix with dynamic rows and static columns.
		 */
		template <unsigned int otherColumns>
		const Matrix<rows, otherColumns>  operator*(const Matrix<columns, otherColumns> & matrix) const
		{
			// Temporary matrix - to prevent necessary values from overwriting.
			Matrix<rows, otherColumns> result;

			for (unsigned int row = 0; row < rows; ++row) {
				for (unsigned int column = 0; column < otherColumns; ++column) {

					double sum = 0.0;

					for (unsigned int k = 0; k < columns ; ++k) {
						sum += _values[row][k] * matrix._values[k][column];
					}

					result._values[row][column] = sum;
				}
			}

			return result;
		}

		/**
		 * Gets back the transposed matrix.
		 *
		 * Transposition of this matrix (function "transpose")is not possible
		 * because we can not dynamically change the number of rows
		 */
		Matrix<columns, rows> Transposed() const
		{
			Matrix<columns, rows> result;

			for (unsigned int i = 0; i < rows; ++i) {
				for (unsigned int j = 0; j < columns; ++j) {
					result._values[j][i] = _values[i][j];
				}
			}

			return result;
		}

		Matrix<rows, columns> Inverse()
		{
			if (rows != columns) {
				// Matrix is not square.
				throw std::range_error("inverse: matrix is not square");
			}
			else if (rows != 2) {
				throw std::runtime_error("inverse: Dimense musi byt 2.");
			}

			// Temporary variable for function inverse matrix.
			unsigned int permutationVector[rows];

			bool err = LUfactorization(permutationVector, _values, columns);
			if (err) {
				// No error.

				inverseOfUpperTriangularMatrix(_values, columns);
				multiplicationInverseUL(permutationVector, _values, columns);
			}
			else {
				throw std::runtime_error("inverse: error");
			}

			return *this;
		}

		// Friends
		template <unsigned int r, unsigned int c> friend class Matrix;

	private:

		/**
		 * Matrix values.
		 */
		double _values[rows][columns];
	};
}

#endif /* MATH_H_ */
