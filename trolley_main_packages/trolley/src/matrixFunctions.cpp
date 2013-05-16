#include <stdio.h>
#include <math.h>
#include "matrixFunctions.h"

using namespace Math;
// LUfactorization algorithm has several parts.
// Part 1 - Finds out the index and the absolute value of the largest element in each column below the diagonal.
// Part 2 - Largest element detected in the previous step is placed on the diagonal (the entire row in the matrix must be swapped, to be provided in the step 4).
// Part 3 - All elements of the given column are divided by the largest element.
// Part 4 - Swap the entire line.
// Part 5 - The elements of the column "column" multiplied by the element from the line maximum are added to the elements of the column "column".


// Only square matrix.
bool Math::LUfactorization (unsigned int *permutationVector, double matrix[2][2], const unsigned int dimension)
{
	if (dimension == 0) {
		return false;
	}
	else if (dimension == 1 && matrix[dimension - 1][dimension - 1] == 0.0) {
		// In case the element is zero, the matrix is singular.

		// Matrix is singular.
		return false;
	}

	// At the last row we dont need to look for the maximum value -> because it is on the diagonal.
	for (unsigned int column = 0; column < dimension - 1; ++column) {

		// Largest value in the column.
		double maxValueInColumn = 0.0;
		// Index of the maximum value in column.
		unsigned int indexMaxValueInColumn = column;

		// Part 1.
		for (unsigned int row = column; row < dimension; ++row) {
			 // To pass the column below the diagonal and find out the max. value.

			// For comparison.
			double compare = fabs(matrix[row][column]);
			if (compare > maxValueInColumn) {
				//index of the maximum value in the column
				indexMaxValueInColumn = row;
				//The new maximum value
				maxValueInColumn = compare;
			}
		}

		// Is storing  the  maximum element into the permutation vector
		permutationVector[column] = indexMaxValueInColumn;

		if (maxValueInColumn == 0) {
			// In case the maximum in the k-th column is zero.

			// Matrix is singular.
			return false;
		}

		// The found maximum is not zero.
		// Part 2.

		if (column != indexMaxValueInColumn) {
			// In case the largest found element is not on the diagonal.

			// Swap the largest element on the diagonal.
			double swap = matrix[indexMaxValueInColumn][column];
			matrix[indexMaxValueInColumn][column] = matrix[column][column];
			matrix[column][column] = swap;  //store on the diagonal
		}

		// Part 3.
		// Divides all numbers in the column below
		// the diagonal by the largest element found in
		// the previous step (and now we have got
		// the largest value on the diagonal).
		double t = -1.0/matrix[column][column];
		for (unsigned int row = column + 1; row < dimension; ++row) {
			matrix[row][column] = matrix[row][column] * t;
		}

		for (unsigned int subColumn = column + 1; subColumn < dimension; ++subColumn) {
			// Part 4.

			if (indexMaxValueInColumn != column) {
				// In case the element was swapped in the matrix, see above, the entire row should be swapped as well.

				// For swapping.
				double swap =  matrix[indexMaxValueInColumn][subColumn];
				matrix[indexMaxValueInColumn][subColumn] =  matrix[column][subColumn];
				matrix[column][subColumn] = swap;
			}

			// Part 5.
			// To the elements of the column "subColumn" are added the elements
			// of the column "column" multiplied by the maximum element of the row.

			// The maximum element of the row.
			t =  matrix[column][subColumn];
			for (unsigned int subRow = column + 1; subRow < dimension; ++subRow) {
				// Addition of columns multiplied by the maximum element of the row.
				matrix[subRow][subColumn] = matrix[subRow][subColumn] + matrix[subRow][column] * t;
			}
		}
	}

	return true;
};



void Math::inverseOfUpperTriangularMatrix (double matrix[2][2], const unsigned int dimension)
{
	// Inverting.
	for (unsigned int column = 0; column < dimension ; ++column) {
		// Move through the columns.

		// Save the inverted diagonal element into the variable.
		double inverseDiagonalElement = 1/matrix[column][column];

		// Put back the inverted number on the diagonal.
		matrix[column][column] = inverseDiagonalElement;

		for (unsigned int row = 0; row < column; ++row) {
			// Multiply the whole column above the diagonal by the inverted value.
			matrix[row][column] *= (-1) * inverseDiagonalElement;
		}

		if (column < dimension)  {
			// In case the "column" is not the last one.

			for (unsigned int subColumn = column + 1; subColumn < dimension; ++subColumn) {
				// Move from the column + 1 up to the last one.

				// Element on the right of the diagonal.
				double t = matrix[column][subColumn];

				matrix[column][subColumn] = 0;

				for (unsigned int subRow = 0; subRow < (column + 1) ; ++subRow) {
					matrix[subRow][subColumn] += t * matrix[subRow][column];
				}
			}
		}
	}
}

void Math::multiplicationInverseUL (unsigned int *permutationVector, double matrix[2][2], const unsigned int dimension)
{
	if (dimension == 1) {
		// In case of rows == 1 the inversion is already complete
		return;
	}

	// Auxiliary array.
	double pwork[dimension];

	for (int column = dimension - 2; column >= 0; --column) {
		for (unsigned int row = column + 1; row < dimension; ++row)  {
			// Store the column below the diagonal into the array.

			pwork[row] = matrix[row][column];
			// Erasing.
			matrix[row][column] = 0.0;
		}

		// The column "subColumn" multiplied by the auxiliary vector value is added to the column "column".
		for (unsigned int subColumn = column + 1; subColumn < dimension; ++subColumn) {
			// Move through the column column + 1 up to dimension - 1

			// Adding.
			for (unsigned int subRow = 0; subRow < dimension; ++subRow) {
				matrix[subRow][column] += matrix[subRow][subColumn] * pwork[subColumn];
			}
		}

		// Index of the swapped row.
		unsigned int permutation = permutationVector[column];
		if ( permutation != static_cast<unsigned int> (column)) {
			// In case the index of the maximum value from the LUfactorization is not on the diagonal.
			for(unsigned int swapRow = 0; swapRow <dimension; ++swapRow) {
				// Swap columns.
				double swap = matrix[swapRow][permutation];
				matrix[swapRow][permutation] = matrix[swapRow][column];
				matrix[swapRow][column] = swap;
			}
		}
	}
}
