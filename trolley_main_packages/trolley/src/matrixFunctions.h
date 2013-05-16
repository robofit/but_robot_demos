#ifndef MATH_PROCEDURAL_MATRIXFUNCTIONS_H
#define MATH_PROCEDURAL_MATRIXFUNCTIONS_H



namespace Math
{
	/**
	 * LU factorization of a matrix using Gaussian elimination
	 */
	bool LUfactorization (unsigned int *permutationVector, double matrix[2][2], const unsigned int dimension);

	/**
	 * Inverse of upper triangular matrix U "ziskane LU faktorizaci"
	 * ! predpoklada probehnuti Dgefa
	 */
	void inverseOfUpperTriangularMatrix (double matrix[2][2], const unsigned int dimension);

    /**
	 * Multiplication inverse matrix U and inverse matrix L
	 * ! predpoklada probehnuti Dgefa a InvU
	 */
	void multiplicationInverseUL (unsigned int *permutationVector, double matrix[2][2], const unsigned int dimension);
}

#endif
