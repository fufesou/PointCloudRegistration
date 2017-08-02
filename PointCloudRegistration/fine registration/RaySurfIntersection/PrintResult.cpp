#include "PrintResult.h"
#include <iostream>
#include "..\ICPType.h"
#include "CreateMatrix.h"
#include "Utility.h"

//*****************************************************************************************
//FUNCTION
void printResult(const double vInitialU, const double vInitialV, const Eigen::Matrix4d &vNNx, const Eigen::Matrix4d &vNNy, const Eigen::Matrix4d &vNNz)
{
	double EndX = createMatrixU(vInitialU) * vNNx * createMatrixV(vInitialV);
	double EndY = createMatrixU(vInitialU) * vNNy * createMatrixV(vInitialV);
	double EndZ = createMatrixU(vInitialU) * vNNz * createMatrixV(vInitialV);

	hiveRegistration::VecColVector3 TestIntPoints;
	TestIntPoints.push_back(hiveRegistration::ColVector3(EndX, EndY, EndZ));
	export2Ply("intersect_point.ply", TestIntPoints);

	std::cout << "x,y,z=" << EndX << std::endl << EndY << std::endl << EndZ << std::endl;
}
