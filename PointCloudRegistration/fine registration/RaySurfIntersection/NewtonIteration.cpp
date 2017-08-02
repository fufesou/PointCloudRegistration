#include "NewtonIteration.h"
#include "CreateMatrix.h"
#include "ComputeData.h"

//*********************************************************************************************
//FUNCTION
bool useNewtonSolveEqution(const boost::tuple<double, double, double> &vCoordinate, const boost::tuple<double, double, double> &vDirection, boost::tuple<double, double, double> &vInitial,
						   const Eigen::Matrix4d &vNNx, const Eigen::Matrix4d &vNNy, const Eigen::Matrix4d &vNNz)
{
	double InitialU = vInitial.get<0>();
	double InitialV = vInitial.get<1>();
	double InitialT = vInitial.get<2>();

	double x0 = vCoordinate.get<0>();
	double y0 = vCoordinate.get<1>();
	double z0 = vCoordinate.get<2>();

	double A = vDirection.get<0>();
	double B = vDirection.get<1>();
	double C = vDirection.get<2>();
	
	double F1 = createMatrixU(InitialU) * vNNx * createMatrixV(InitialV) - x0 - A * InitialT;
	double F2 = createMatrixU(InitialU) * vNNy * createMatrixV(InitialV) - y0 - B * InitialT;
	double F3 = createMatrixU(InitialU) * vNNz * createMatrixV(InitialV) - z0 - C * InitialT;

	int RepeatCount = 0;
	while ( (fabs(F1) > EPSILON) || (fabs(F2) > EPSILON) || (fabs(F3) > EPSILON) )
	{
		Eigen::Matrix<double , 3, 1> MatrixComputeData = computeData(vNNx, vNNy, vNNz, InitialU, InitialV, InitialT, A, B, C, F1, F2, F3);

		InitialU = MatrixComputeData(0, 0);
		InitialV = MatrixComputeData(1, 0);
		InitialT = MatrixComputeData(2, 0);

		F1 = createMatrixU(InitialU) * vNNx * createMatrixV(InitialV) - x0 - A * InitialT;
		F2 = createMatrixU(InitialU) * vNNy * createMatrixV(InitialV) - y0 - B * InitialT;
		F3 = createMatrixU(InitialU) * vNNz * createMatrixV(InitialV) - z0 - C * InitialT;

		++RepeatCount;
		if (RepeatCount > 10) return false;

		_ASSERT(RepeatCount < 1000);
	}

	if ( (0<InitialU) && (InitialU<1) && (0<InitialV) && (InitialV<1) )
	{
		 vInitial.get<0>() = InitialU;
		 vInitial.get<1>() = InitialV;
		 vInitial.get<2>() = InitialT;
		return true;
	}
	else
	{
		return false;
	}
}