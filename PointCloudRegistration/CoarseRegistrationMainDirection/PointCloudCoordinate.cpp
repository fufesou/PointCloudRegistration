#include "PointCloudCoordinate.h"
#include <algorithm>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include "PointCloud.h"
#include "RegUtilityFunctions.h"

using namespace hiveRegistration;

//*********************************************************************************
//FUNCTION:
CPointCloudCoordinate::CPointCloudCoordinate()
: m_pPointCloud(NULL)
{

}

//*********************************************************************************
//FUNCTION:
void CPointCloudCoordinate::refresh(const hivePointCloud::CPointCloud* vPointCloud)
{
	_ASSERT(vPointCloud);
	if (m_pPointCloud != vPointCloud)
	{
		m_pPointCloud = vPointCloud;
		__setCoordinate();
	}
}

//*********************************************************************************
//FUNCTION:
void CPointCloudCoordinate::__setCoordinate()
{
	compute3DCentroid(m_pPointCloud->getPosPointer(), m_pPointCloud->getNumPoints(), m_OriginPoint);
	__computeAxisDir();
}

//*********************************************************************************
//FUNCTION:
void CPointCloudCoordinate::__computeAxisDir()
{
	Eigen::Matrix3d ConvarianceMatrix;
	__buildConvarianceMatrix(ConvarianceMatrix);
	Eigen::EigenSolver<Eigen::Matrix3d> EigenSolver(ConvarianceMatrix);
	Eigen::Vector3d EigenValues	 = EigenSolver.eigenvalues().real();
	Eigen::Matrix3d EigenVectors = EigenSolver.eigenvectors().real();
	__sortEigenValuesAndVectors(EigenValues, EigenVectors);

	m_AxisDir.col(0) = EigenVectors.col(2);
	m_AxisDir.col(1) = EigenVectors.col(1);
	//m_AxisDir.col(2) = (EigenVectors.col(2)).cross(EigenVectors.col(1));
	m_AxisDir.col(2) = EigenVectors.col(0);
	(m_AxisDir.col(0)).normalize();
	(m_AxisDir.col(1)).normalize();
	(m_AxisDir.col(2)).normalize();
}

//*********************************************************************************
//FUNCTION:
void CPointCloudCoordinate::__buildConvarianceMatrix(Eigen::Matrix3d& voConvarianceMatrix)
{
	voConvarianceMatrix.setZero();
	const unsigned int NumPoints = m_pPointCloud->getNumPoints();
	const double* pPointPos = m_pPointCloud->getPosPointer();

	for (unsigned int i=0; i<NumPoints; ++i)
	{
		for (unsigned int k=0; k<3; ++k)
		{
			for (unsigned int m=0; m<3; ++m)
			{
				voConvarianceMatrix(k, m) += (*(pPointPos + k) - m_OriginPoint(k, 0)) * (*(pPointPos + m) - m_OriginPoint(m, 0));
			}
		}
		pPointPos += 3;
	}
}

//*********************************************************************************
//FUNCTION:
void CPointCloudCoordinate::__sortEigenValuesAndVectors(Eigen::Vector3d& vioValues, Eigen::Matrix3d& vioVectors)
{
	int Order[3] = { 0, 1, 2 };

	if (vioValues(Order[1], 0) < vioValues(Order[0], 0))
	{
		std::swap(Order[1], Order[0]);
	}
	if (vioValues(Order[2], 0) < vioValues(Order[1], 0))
	{
		std::swap(Order[2], Order[1]);
		if (vioValues(Order[1], 0) < vioValues(Order[0], 0))
		{
			std::swap(Order[1], Order[0]);
		}
	}

	Eigen::Vector3d TmpVec = vioValues;
	Eigen::Matrix3d TmpMat = vioVectors;

	vioValues << TmpVec(Order[0], 0), TmpVec(Order[1], 0), TmpVec(Order[2], 0);
	vioVectors.col(0) = TmpMat.col(Order[0]);
	vioVectors.col(1) = TmpMat.col(Order[1]);
	vioVectors.col(2) = TmpMat.col(Order[2]);
}