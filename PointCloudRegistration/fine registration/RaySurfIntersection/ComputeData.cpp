#include "ComputeData.h"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/LU>
#include "ComputeDerivative.h"

//**************************************************************************************
//FUNCTION
Eigen::Matrix<double , 3, 1> computeData(const Eigen::Matrix4d &vNNx, const Eigen::Matrix4d &vNNy, const Eigen::Matrix4d &vNNz, double vInitialU, double vInitialV, double vInitialT, 
										 const double vA, const double vB, const double vC, const double vF1, const double vF2, const double vF3)
{
	double A00 = computeDerivativeU(vNNx, vInitialU, vInitialV);
	double A01 = computeDerivativeV(vNNx, vInitialU, vInitialV);
	double A02 = -vA;

	double A10 = computeDerivativeU(vNNy, vInitialU, vInitialV);
	double A11 = computeDerivativeV(vNNy, vInitialU, vInitialV);
	double A12 = -vB;

	double A20 = computeDerivativeU(vNNz, vInitialU, vInitialV);
	double A21 = computeDerivativeV(vNNz, vInitialU, vInitialV);		
	double A22 = -vC;

	Eigen::Vector3d PreF(-vF1, -vF2, -vF3);

	Eigen::Matrix3d DF;
	DF << A00, A01, A02, A10, A11, A12, A20, A21, A22;

	Eigen::Vector3d NewX = DF.inverse() * PreF + Eigen::Vector3d(vInitialU, vInitialV, vInitialT);

	return NewX;
}
