#include "Bicubic.h"
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <boost\tuple\tuple.hpp>
#include "ControlPointsData.h"
#include "ICPType.h"
#include "NewtonIteration.h"
#include "PrintResult.h"

using namespace hiveRegistration;

Bicubic::CBicubic::CBicubic()
{
}

Bicubic::CBicubic::~CBicubic()
{
}

//*********************************************************************************
//FUNCTION:
void Bicubic::CBicubic::computeNormal( double vU, double vV, const Eigen::Matrix4d vControlPoints[], hiveRegistration::ColVector3& voNormal ) const
{
	Eigen::Matrix4d MatrixN;
	MatrixN <<  1,  4,  1,  0,
		-3,  0,  3,  0,
		3, -6,  3,  0, 
		-1,  3, -3,  1;

	MatrixN = MatrixN * (1.0 / 6.0);

	Eigen::Matrix<double, 1, 4> OriginalU(1, vU, vU*vU, vU*vU*vU);
	Eigen::Matrix<double, 1, 4> OriginalV(1, vV, vV*vV, vV*vV*vV);
	Eigen::Matrix<double, 1, 4> DerivationU(0, 1, 2*vU, 3*vU*vU);
	Eigen::Matrix<double, 1, 4> DerivationV(0, 1, 2*vV, 3*vV*vV);

	double XSurfaceDerivationU = DerivationU	* MatrixN * vControlPoints[0] * MatrixN.transpose() * OriginalV.transpose();
	double XSurfaceDerivationV = OriginalU		* MatrixN * vControlPoints[0] * MatrixN.transpose() * DerivationV.transpose();
	double YSurfaceDerivationU = DerivationU	* MatrixN * vControlPoints[1] * MatrixN.transpose() * OriginalV.transpose();
	double YSurfaceDerivationV = OriginalU		* MatrixN * vControlPoints[1] * MatrixN.transpose() * DerivationV.transpose();
	double ZSurfaceDerivationU = DerivationU	* MatrixN * vControlPoints[2] * MatrixN.transpose() * OriginalV.transpose();
	double ZSurfaceDerivationV = OriginalU		* MatrixN * vControlPoints[2] * MatrixN.transpose() * DerivationV.transpose();

	ColVector3 SU(XSurfaceDerivationU, YSurfaceDerivationU, ZSurfaceDerivationU);
	ColVector3 SV(XSurfaceDerivationV, YSurfaceDerivationV, ZSurfaceDerivationV);

	ColVector3 SU_SV;
	SU_SV << SU(1, 0)*SV(2, 0)-SV(1, 0)*SU(2, 0), SU(0, 0)*SV(2, 0)-SV(0, 0)*SU(2, 0), SU(0, 0)*SV(1, 0)-SV(0, 0)*SU(1, 0);

	double Module = sqrt(SU_SV(0, 0)*SU_SV(0, 0) + SU_SV(1, 0)*SU_SV(1, 0) + SU_SV(2, 0)*SU_SV(2, 0));

	voNormal = SU_SV / Module;
}

//*********************************************************************************
//FUNCTION:
void Bicubic::CBicubic::computePrincipleCurvatures(double vU, double vV, const Eigen::Matrix4d vControlPoints[], std::pair<double, double>& voPrincipleCurvatures) const
{
	Eigen::Matrix4d MatrixN;
	MatrixN <<  1,  4,  1,  0,
		-3,  0,  3,  0,
		3, -6,  3,  0, 
		-1,  3, -3,  1;

	Eigen::Matrix<double, 1, 4> OriginalU(1, vU, vU*vU, vU*vU*vU);
	Eigen::Matrix<double, 1, 4> OriginalV(1, vV, vV*vV, vV*vV*vV);
	Eigen::Matrix<double, 1, 4> DerivationU(0, 1, 2*vU, 3*vU*vU);
	Eigen::Matrix<double, 1, 4> DerivationV(0, 1, 2*vV, 3*vV*vV);
	Eigen::Matrix<double, 1, 4> DerivationUU(0, 0, 2, 6*vU);
	Eigen::Matrix<double, 1, 4> DerivationVV(0, 0, 2, 6*vV);

	double XSurfaceDerivationU		= (DerivationU	* MatrixN * vControlPoints[0] * MatrixN.transpose() * OriginalV.transpose() / 36.0)(0, 0);
	double YSurfaceDerivationU		= (DerivationU	* MatrixN * vControlPoints[1] * MatrixN.transpose() * OriginalV.transpose() / 36.0)(0, 0);
	double ZSurfaceDerivationU		= (DerivationU	* MatrixN * vControlPoints[2] * MatrixN.transpose() * OriginalV.transpose() / 36.0)(0, 0);

	double XSurfaceDerivationV		= (OriginalU	* MatrixN * vControlPoints[0] * MatrixN.transpose() * DerivationV.transpose() / 36.0)(0, 0);
	double YSurfaceDerivationV		= (OriginalU	* MatrixN * vControlPoints[1] * MatrixN.transpose() * DerivationV.transpose() / 36.0)(0, 0);
	double ZSurfaceDerivationV		= (OriginalU	* MatrixN * vControlPoints[2] * MatrixN.transpose() * DerivationV.transpose() / 36.0)(0, 0);

	double XSurfaceDerivationUU		= (DerivationUU	* MatrixN * vControlPoints[0] * MatrixN.transpose() * OriginalV.transpose() / 36.0)(0, 0);
	double YSurfaceDerivationUU		= (DerivationUU	* MatrixN * vControlPoints[1] * MatrixN.transpose() * OriginalV.transpose() / 36.0)(0, 0);
	double ZSurfaceDerivationUU		= (DerivationUU	* MatrixN * vControlPoints[2] * MatrixN.transpose() * OriginalV.transpose() / 36.0)(0, 0);

	double XSurfaceDerivationVV		= (OriginalU	* MatrixN * vControlPoints[0] * MatrixN.transpose() * DerivationVV.transpose() / 36.0)(0, 0);
	double YSurfaceDerivationVV		= (OriginalU	* MatrixN * vControlPoints[1] * MatrixN.transpose() * DerivationVV.transpose() / 36.0)(0, 0);
	double ZSurfaceDerivationVV		= (OriginalU	* MatrixN * vControlPoints[2] * MatrixN.transpose() * DerivationVV.transpose() / 36.0)(0, 0);

	double XSurfaceDerivationUV		= (DerivationU	* MatrixN * vControlPoints[0] * MatrixN.transpose() * DerivationV.transpose() / 36.0)(0, 0);
	double YSurfaceDerivationUV		= (DerivationU	* MatrixN * vControlPoints[1] * MatrixN.transpose() * DerivationV.transpose() / 36.0)(0, 0);
	double ZSurfaceDerivationUV		= (DerivationU	* MatrixN * vControlPoints[2] * MatrixN.transpose() * DerivationV.transpose() / 36.0)(0, 0);

	ColVector3 SU(XSurfaceDerivationU, YSurfaceDerivationU, ZSurfaceDerivationU);
	ColVector3 SV(XSurfaceDerivationV, YSurfaceDerivationV, ZSurfaceDerivationV);
	ColVector3 SUU(XSurfaceDerivationUU, YSurfaceDerivationUU, ZSurfaceDerivationUU);
	ColVector3 SVV(XSurfaceDerivationVV, YSurfaceDerivationVV, ZSurfaceDerivationVV);
	ColVector3 SUV(XSurfaceDerivationUV, YSurfaceDerivationUV, ZSurfaceDerivationUV);

	ColVector3 Normal;
	Normal << SU(1, 0)*SV(2, 0)-SV(1, 0)*SU(2, 0), SU(0, 0)*SV(2, 0)-SV(0, 0)*SU(2, 0), SU(0, 0)*SV(1, 0)-SV(0, 0)*SU(1, 0);
	Normal.normalize();
	
	double E = SU.dot(SU);
	double F = SU.dot(SV);
	double G = SV.dot(SV);
	double M = Normal.dot(SUV);
	double L = Normal.dot(SUU);
	double N = Normal.dot(SVV);

	double K = (L*N - M*M) / (2 * (E*G - F*F));
	double H = (E*N - 2*F*M + G*L) / (2 * (E*G - F*F));

	voPrincipleCurvatures.first		= H - sqrt(H*H - K);
	voPrincipleCurvatures.second	= H + sqrt(H*H - K);
}

//*********************************************************************************
//FUNCTION:
void Bicubic::CBicubic::compute3DPointPosByUV( double vU, double vV, const Eigen::Matrix4d vControlPoints[], hiveRegistration::ColVector3& voPos ) const
{
	Eigen::Matrix4d MatrixN;
	MatrixN <<  1,  4,  1,  0,
		-3,  0,  3,  0,
		3, -6,  3,  0, 
		-1,  3, -3,  1;
	MatrixN = MatrixN * (1.0/6.0);

	Eigen::Matrix<double, 1, 4> OriginalU(1, vU, vU*vU, vU*vU*vU);
	Eigen::Matrix<double, 1, 4> OriginalV(1, vV, vV*vV, vV*vV*vV);

	voPos(0, 0) = OriginalU * MatrixN * vControlPoints[0] * MatrixN.transpose() * OriginalV.transpose();
	voPos(1, 0) = OriginalU * MatrixN * vControlPoints[1] * MatrixN.transpose() * OriginalV.transpose();
	voPos(2, 0) = OriginalU * MatrixN * vControlPoints[2] * MatrixN.transpose() * OriginalV.transpose();
}

//*********************************************************************************
//FUNCTION:
void Bicubic::CBicubic::intersectRay( const Eigen::Matrix4d vCtrlPoints[], const hiveRegistration::ColVector3& vRayOrigin, const hiveRegistration::ColVector3& vRayDir, hiveRegistration::ColVector2& voEndMatrix, bool& voIntersected ) const
{
	const Eigen::Matrix<double, 4, 4> DX = vCtrlPoints[0];
	const Eigen::Matrix<double, 4, 4> DY = vCtrlPoints[1];
	const Eigen::Matrix<double, 4, 4> DZ = vCtrlPoints[2];

	boost::tuple<double, double, double> Coordinate;

	Coordinate.get<0>() = vRayOrigin(0, 0);
	Coordinate.get<1>() = vRayOrigin(1, 0);
	Coordinate.get<2>() = vRayOrigin(2, 0);

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
		printResult(InitialU, InitialV, NNx, NNy, NNz);

		voEndMatrix << InitialU,InitialV;	    

		voIntersected = true;
	}
	else
	{
		voIntersected = false;
	}
}
