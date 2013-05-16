/*
 * kalmanFilter.cpp
 *
 *  Created on: Feb 19, 2013
 *      Author: tomaskolo
 */

#include "kalmanFilter.h"

using namespace Math;

Math::KalmanFilter::KalmanFilter(double dt, double processNoisePSD, double measurementNoiseVariance)
{
	double Xvalue[4][1] = {
			{0},
			{0},
			{0},
			{0}};
	X = Matrix<4,1>(Xvalue);

	double Pvalue[4][4] = {
			{0, 0, 0, 0},
			{0, 0, 0, 0},
			{0, 0, 0, 0},
			{0, 0, 0, 0}};
	P = Matrix<4,4>(Pvalue);

	double Fvalue[4][4] = {
			{1, 0, dt, 0},
			{0, 1, 0, dt},
			{0, 0, 1, 0},
			{0, 0, 0, 1}};
	F = Matrix<4,4>(Fvalue);


	double Qvalue[4][4] = {
			{0, 0, 0, 0},
			{0, 0, 0, 0},
			{0, 0, processNoisePSD * processNoisePSD, 0},
			{0, 0, 0, processNoisePSD * processNoisePSD}};
	Q = Matrix<4,4>(Qvalue);


	double Hvalue[2][4] = {
			{1, 0, 0, 0},
			{0, 1, 0, 0}};
	H = Matrix<2,4>(Hvalue);


	double Rvalue[2][2] = {
			{measurementNoiseVariance, 0},
			{0, measurementNoiseVariance}};
	R = Matrix<2,2>(Rvalue);
}

void Math::KalmanFilter::Predict()
{
	X0 = F * X;
	P0 = F * P * F.Transposed() + Q;
}

void Math::KalmanFilter::Correct(const Matrix<2,1> & Z)
{
	Matrix<2,2> S = H * P0 * H.Transposed() + R;

	Matrix<4,2> K = P0 * (H.Transposed() * S.Inverse());

	X = X0 + K * (Z - (H * X0));

	double Ivalue[4][4] = {
			{1, 0, 0, 0},
			{0, 1, 0, 0},
			{0, 0, 1, 0},
			{0, 0, 0, 1}};
	Matrix<4,4> I(Ivalue);

	P = (I - K * H) * P0;
}

void Math::KalmanFilter::Correct()
{
	//Matrix<2,2> S = H * P0 * H.Transposed() + R;

	Matrix<2,1> Z;
	Matrix<4,2> K;

	X = X0 + K * (Z - (H * X0));

	double Ivalue[4][4] = {
			{1, 0, 0, 0},
			{0, 1, 0, 0},
			{0, 0, 1, 0},
			{0, 0, 0, 1}};
	Matrix<4,4> I(Ivalue);

	P = (I - K * H) * P0;
}

void Math::KalmanFilter::SetX(const Matrix<4,1> & X)
{
	this->X = X;
}

const Matrix<4,1> & Math::KalmanFilter::GetX() const
{
	return X;
}
