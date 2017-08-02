#include "Intersection.h"
#include <iostream>
#include <boost/tuple/tuple.hpp>
#include ".\ICPType.h"
#include "CreateMatrix.h"
#include "PrintResult.h"
#include "NewtonIteration.h"

//*************************************************************************************
//FUNCTION
bool intersect(const Eigen::Matrix<double, 4, 4> vCtrlPnts[3], const Eigen::Matrix<double, 3, 1> &vRayPnt, const Eigen::Matrix<double, 3, 1> &vRayDir, Eigen::Matrix<double, 2, 1> &voUV, hiveRegistration::ColVector3 &voIntersectPoint)
{
	if (!intersect(vCtrlPnts, vRayPnt, vRayDir, voUV)) return false;

	const Eigen::Matrix<double, 4, 4>& DX = vCtrlPnts[0];
	const Eigen::Matrix<double, 4, 4>& DY = vCtrlPnts[1];
	const Eigen::Matrix<double, 4, 4>& DZ = vCtrlPnts[2];

	static Eigen::Matrix<double, 4, 4> N;
	N <<  1,  4,  1, 0,
		-3,  0 , 3, 0,
		3, -6,  3, 0,
		-1,  3, -3, 1;
	N = N * (1.0/6.0);

	Eigen::Matrix4d NNx = N * DX * (N.transpose());
	Eigen::Matrix4d NNy = N * DY * (N.transpose());
	Eigen::Matrix4d NNz = N * DZ * (N.transpose());

	double EndX = createMatrixU(voUV(0, 0)) * NNx * createMatrixV(voUV(1, 0));
	double EndY = createMatrixU(voUV(0, 0)) * NNy * createMatrixV(voUV(1, 0));
	double EndZ = createMatrixU(voUV(0, 0)) * NNz * createMatrixV(voUV(1, 0));

	voIntersectPoint << EndX, EndY, EndZ;

	return true;
}

//*********************************************************************************
//FUNCTION:
bool intersect(const Eigen::Matrix<double, 4, 4> vCtrlPnts[3], const Eigen::Matrix<double, 3, 1> &vRayPnt, const Eigen::Matrix<double, 3, 1> &vRayDir, Eigen::Matrix<double, 2, 1> &voUV)
{
	const Eigen::Matrix<double, 4, 4>& DX = vCtrlPnts[0];
	const Eigen::Matrix<double, 4, 4>& DY = vCtrlPnts[1];
	const Eigen::Matrix<double, 4, 4>& DZ = vCtrlPnts[2];

	boost::tuple<double, double, double> Coordinate;

	Coordinate.get<0>() = vRayPnt(0, 0);
	Coordinate.get<1>() = vRayPnt(1, 0);
	Coordinate.get<2>() = vRayPnt(2, 0);

	boost::tuple<double, double, double> Direction;

	Direction.get<0>() = vRayDir(0, 0);
	Direction.get<1>() = vRayDir(1, 0);
	Direction.get<2>() = vRayDir(2, 0);

	boost::tuple<double, double, double> Initial(0, 0, 0);

	static Eigen::Matrix<double, 4, 4> N;
	N <<  1,  4,  1, 0,
		-3,  0 , 3, 0,
		3, -6,  3, 0,
		-1,  3, -3, 1;
	N = N * (1.0/6.0);

	Eigen::Matrix4d NNx = N * DX * (N.transpose());
	Eigen::Matrix4d NNy = N * DY * (N.transpose());
	Eigen::Matrix4d NNz = N * DZ * (N.transpose());

	bool IsHaveSolution = useNewtonSolveEqution(Coordinate, Direction, Initial, NNx, NNy, NNz);

	double InitialU = Initial.get<0>();
	double InitialV = Initial.get<1>();

	if (IsHaveSolution)
	{		
		voUV << InitialU, InitialV;	    
		return true;
	}
	else
	{
#ifdef _DEBUG
		//std::cout << "the total  repeat number is:" << TotalRepeatCount << std::endl;
#endif
		return false;
	}
}
