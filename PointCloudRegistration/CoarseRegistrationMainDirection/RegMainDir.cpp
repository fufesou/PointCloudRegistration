#include "RegMainDir.h"
#include <algorithm>
#include <Eigen/LU>
#include "ProductFactoryData.h"
#include "PointCloud.h"
#include "ControlParameters.h"
#include "ICPMacros.h"
#include "RegUtilityFunctions.h"
#include "PointCloudCoordinate.h"

#include <fstream>

using namespace hiveRegistration;

CRegMainDirect::CRegMainDirect()
: m_pSrcPointCloud(NULL)
, m_pTgtPointCloud(NULL)
, m_FitRes("NotFit")
{
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyCoincidentThreshold(), 0.95);
}

//*********************************************************************************
//FUNCTION:
bool CRegMainDirect::fit(const hivePointCloud::CPointCloud& vSrc, const hivePointCloud::CPointCloud& vTgt, Eigen::Matrix3d& voR, ColVector3& voT, double& voCoincidentCoeff)
{
	voCoincidentCoeff = 0.0;

	__refreshData(vSrc, vTgt);
	return __fit(voR, voT, voCoincidentCoeff);
}

//*********************************************************************************
//FUNCTION:
void CRegMainDirect::__computeCoordTransformation(const CoordDataType& vSrcCoord, const CoordDataType& vTgtCoord, Eigen::Matrix3d& voR, ColVector3& voT)
{
	voR = vTgtCoord.second * (vSrcCoord.second).inverse();
	voT = vTgtCoord.first - voR * vSrcCoord.first;
}

//*********************************************************************************
//FUNCTION:
bool CRegMainDirect::__fit(Eigen::Matrix3d& voR, ColVector3& voT, double& vioCoincidentCoeff)
{
	const CoordDataType& TgtCoordData = m_TgtCoordData;
	CoordDataType SrcCoordData(m_SrcCoordData);

	for (int i=0; i<8; ++i)
	{
		SrcCoordData.second.col(0) = ( ((  i    & 1) << 1) - 1 ) * m_SrcCoordData.second.col(0);
		SrcCoordData.second.col(1) = ( (((i>>1) & 1) << 1) - 1 ) * m_SrcCoordData.second.col(1);
		SrcCoordData.second.col(2) = ( (((i>>2) & 1) << 1) - 1 ) * m_SrcCoordData.second.col(2);

		if ( __fitOneDir(SrcCoordData, TgtCoordData, voR, voT, vioCoincidentCoeff) )
		{
			return true;
		}
	}

	return false;
}

//*********************************************************************************
//FUNCTION:
double CRegMainDirect::__computeVolume(const Eigen::Matrix<double, 3, 2>& vBox)
{
	_ASSERT(vBox(0, 0) < vBox(0, 1) && vBox(1, 0) < vBox(1, 1) && vBox(2, 0) < vBox(2, 1));
	return (vBox(0, 1) - vBox(0, 0)) * (vBox(1, 1) - vBox(1, 0)) * (vBox(2, 1) - vBox(2, 0));
}

//*********************************************************************************
//FUNCTION:
double CRegMainDirect::__computeCoincidentVolume(const Eigen::Matrix<double, 3, 2>& vBoxA, const Eigen::Matrix<double, 3, 2>& vBoxB)
{
	_ASSERT(vBoxA(0, 0) < vBoxA(0, 1) && vBoxA(1, 0) < vBoxA(1, 1) && vBoxA(2, 0) < vBoxA(2, 1));
	_ASSERT(vBoxB(0, 0) < vBoxB(0, 1) && vBoxB(1, 0) < vBoxB(1, 1) && vBoxB(2, 0) < vBoxB(2, 1));

	const double NoCoincidentVolume = 0;
	double BoxDataArr[12] = 
	{
		vBoxA(0, 0), vBoxA(0, 1), vBoxB(0, 0), vBoxB(0, 1),
		vBoxA(1, 0), vBoxA(1, 1), vBoxB(1, 0), vBoxB(1, 1),
		vBoxA(2, 0), vBoxA(2, 1), vBoxB(2, 0), vBoxB(2, 1)
	};

	std::sort(BoxDataArr + 0, BoxDataArr + 4);
	std::sort(BoxDataArr + 4, BoxDataArr + 8);
	std::sort(BoxDataArr + 8, BoxDataArr + 12);

	if ( (BoxDataArr[1] - vBoxA(0, 1)) > EPSILON || (BoxDataArr[1] - vBoxB(0, 1)) > EPSILON ||
		 (BoxDataArr[5] - vBoxA(1, 1)) > EPSILON || (BoxDataArr[5] - vBoxB(1, 1)) > EPSILON ||
		 (BoxDataArr[9] - vBoxA(2, 1)) > EPSILON || (BoxDataArr[9] - vBoxB(2, 1)) > EPSILON )
	{
		return NoCoincidentVolume;
	}

	return (BoxDataArr[2] - BoxDataArr[1]) * (BoxDataArr[6] - BoxDataArr[5]) * (BoxDataArr[10] - BoxDataArr[9]);
}

//*********************************************************************************
//FUNCTION:
double CRegMainDirect::__computeCurCoincidentCoeff(const Eigen::Matrix<double, 3, 2>& vRegionA, const Eigen::Matrix<double, 3, 2>& vRegionB)
{
	const double CoincidentVolume = __computeCoincidentVolume(vRegionA, vRegionB);		// FIXME: 如果两个立方体的z坐标都是0，则计算出错
	const double VolumeBoxA       = __computeVolume(vRegionA);
	const double VolumeBoxB       = __computeVolume(vRegionB);
	return CoincidentVolume * CoincidentVolume / (VolumeBoxA * VolumeBoxB);
}

//*********************************************************************************
//FUNCTION:
void CRegMainDirect::__transformatePointCloud(const double* vOriginPos, const unsigned int vNumPoint, const Eigen::Matrix3d& vR, const ColVector3& vT, double* voNewPos)
{
	const int NumPoint = static_cast<int>(vNumPoint);

#pragma omp parallel for
	for (int i=0; i<NumPoint; ++i)
	{
		*(voNewPos + i*3 + 0) = vR(0, 0) * (*(vOriginPos + i*3 + 0)) + vR(0, 1) * (*(vOriginPos + i*3 + 1)) + vR(0, 2) * (*(vOriginPos + i*3 + 2)) + vT(0, 0);
		*(voNewPos + i*3 + 1) = vR(1, 0) * (*(vOriginPos + i*3 + 0)) + vR(1, 1) * (*(vOriginPos + i*3 + 1)) + vR(1, 2) * (*(vOriginPos + i*3 + 2)) + vT(1, 0);
		*(voNewPos + i*3 + 2) = vR(2, 0) * (*(vOriginPos + i*3 + 0)) + vR(2, 1) * (*(vOriginPos + i*3 + 1)) + vR(2, 2) * (*(vOriginPos + i*3 + 2)) + vT(2, 0);
	}
}

//*********************************************************************************
//FUNCTION:
void CRegMainDirect::__refreshData(const hivePointCloud::CPointCloud& vSrc, const hivePointCloud::CPointCloud& vTgt)
{
	CPointCloudCoordinate PointCloudCoordComputer;

	if (m_pSrcPointCloud != (&vSrc))
	{
		m_pSrcPointCloud = &vSrc;
		PointCloudCoordComputer.refresh(m_pSrcPointCloud);
		m_SrcCoordData = std::make_pair(PointCloudCoordComputer.getOriginPoint(), PointCloudCoordComputer.getAxisDir());
	}

	if (m_pTgtPointCloud != (&vTgt))
	{
		m_pTgtPointCloud = &vTgt;
		PointCloudCoordComputer.refresh(m_pTgtPointCloud);
		m_TgtCoordData = std::make_pair(PointCloudCoordComputer.getOriginPoint(), PointCloudCoordComputer.getAxisDir());
	}
}

//*********************************************************************************
//FUNCTION:
bool CRegMainDirect::__fitOneDir(const CoordDataType& vSrcCoord, const CoordDataType& vTgtCoord, Eigen::Matrix3d& voR, ColVector3& voT, double& vioCoincidentCoeff)
{
	double* pNewSrcPointPos = new double[m_pSrcPointCloud->getNumPoints() * 3];

	Eigen::Matrix3d TmpRot;
	ColVector3 TmpTra;
	__computeCoordTransformation(vSrcCoord, vTgtCoord, TmpRot, TmpTra);
	__transformatePointCloud(m_pSrcPointCloud->getPosPointer(), m_pSrcPointCloud->getNumPoints(), TmpRot, TmpTra, pNewSrcPointPos);

	std::ofstream OutFile("T000.txt");
	OutFile << "origin src" << "\t\t\t\t" << "new src" << "\t\t\t\t" << "tgt" << std::endl;
	const double* pSrcPos = m_pSrcPointCloud->getPosPointer();
	const double* pTgtPos = m_pTgtPointCloud->getPosPointer();
	for (int i=0; i<m_pSrcPointCloud->getNumPoints(); ++i)
	{
		for (int k=0; k<3; ++k)
		{
			OutFile << *(pSrcPos + i*3 + k) << "\t\t\t\t" << *(pNewSrcPointPos + i*3 + k) << "\t\t\t\t" << *(pTgtPos + i*3 + k) << std::endl;
		}
		OutFile << std::endl;
	}
	OutFile.close();

	Eigen::Matrix<double, 3, 2> BoxA;
	Eigen::Matrix<double, 3, 2> BoxB;
	computeRegion<3>(pNewSrcPointPos, m_pSrcPointCloud->getNumPoints(), BoxA);
	computeRegion<3>(m_pTgtPointCloud->getPosPointer(), m_pSrcPointCloud->getNumPoints(), BoxB);

	delete [] pNewSrcPointPos;

	double CurCoincidentCoeff = __computeCurCoincidentCoeff(BoxA, BoxB);
	if (CurCoincidentCoeff > vioCoincidentCoeff)
	{
		voR = TmpRot;
		voT = TmpTra;
		vioCoincidentCoeff = CurCoincidentCoeff;
	}

	double CoincidentThreshold;
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyCoincidentThreshold(), CoincidentThreshold);
	_ASSERT(DumpRes);

	return (CurCoincidentCoeff > CoincidentThreshold);
}