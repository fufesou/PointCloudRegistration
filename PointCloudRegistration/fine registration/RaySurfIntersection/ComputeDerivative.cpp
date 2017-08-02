#include "ComputeDerivative.h"

//*******************************************************************************************************
//FUNCTION
double computeDerivativeU(const Eigen::Matrix4d &vTempMatrix, const double vParaU, const double vParaV)
{
	 double Result = (vTempMatrix(1,0) + 2 * vParaU * vTempMatrix(2,0) + 3 * vParaU * vParaU * vTempMatrix(3,0)) +
		              vParaV * (vTempMatrix(1,1) + 2 * vParaU * vTempMatrix(2,1) + 3 * vParaU * vParaU * vTempMatrix(3,1)) +
		              vParaV * vParaV * (vTempMatrix(1,2) + 2 * vParaU * vTempMatrix(2,2) + 3 * vParaU * vParaU*vTempMatrix(3,2)) +
		              vParaV * vParaV * vParaV * (vTempMatrix(1,3) + 2 * vParaU*vTempMatrix(2,3) + 3 * vParaU * vParaU * vTempMatrix(3,3));

	 return Result;
}

//********************************************************************************************************
//FUNCTION
double computeDerivativeV(const Eigen::Matrix4d &vTempMatrix, const double vParaU, const double vParaV)
{
	double  Result =  (vTempMatrix(0,1) + vParaU * vTempMatrix(1,1) + vParaU * vParaU * vTempMatrix(2,1) + vParaU * vParaU * vParaU * vTempMatrix(3,1)) +
	                  2 * vParaV * (vTempMatrix(0,2) + vParaU * vTempMatrix(1,2) + vParaU * vParaU * vTempMatrix(2,2) + vParaU * vParaU * vParaU * vTempMatrix(3,2)) +
		              3 * vParaV * vParaV * (vTempMatrix(0,3) + vParaU * vTempMatrix(1,3) + vParaU * vParaU * vTempMatrix(2,3) + vParaU * vParaU * vParaU*vTempMatrix(3,3));

	return Result;

}