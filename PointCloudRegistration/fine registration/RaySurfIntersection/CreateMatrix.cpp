#include "CreateMatrix.h"

//*********************************************************************************
//FUNCTION
Eigen::Matrix<double,1,4> createMatrixU(const double vParaU)
{
	Eigen::Matrix<double,1,4> MatrixU;
	MatrixU << 1, vParaU, vParaU*vParaU, vParaU*vParaU*vParaU;

	return MatrixU;
}

//***********************************************************************************
//FUNCTION
Eigen::Matrix<double,4,1> createMatrixV(const double vParaV)
{
	Eigen::Matrix<double,4,1> MatrixV;
	MatrixV << 1, vParaV, vParaV*vParaV, vParaV*vParaV*vParaV;

	return MatrixV;
}