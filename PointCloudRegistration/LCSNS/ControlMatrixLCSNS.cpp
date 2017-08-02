#include "ControlMatrixLCSNS.h"
#include <time.h>
#include <cfloat>
#include <Eigen/Geometry>
#include "HiveCommon.h"
#include "PointCloud.h"
#include "ProductFactoryData.h"
#include "ControlParameters.h"
#include "ICPMacros.h"
#include "KNNSearch.h"
#include "CommonLCSNSSubset.hpp"
#include "RegUtilityFunctions.h"
#include "RegMath.h"
#include "BaseSampler.h"
#include "ICPType.h"
#include "ControlMatrixData.h"
#include "Bicubic.h"
#include "UniqueData.h"

#ifdef _DEBUG
#include "TestUnitity.h"
#include <fstream>
#endif

using namespace hiveRegistration;

CControlMatrixLCSNS::CControlMatrixLCSNS(void)
	: m_pPointCloud(NULL)
{
	PTR_CONTROL_PARAMS->setIfNotExist(getKeySizeCtrlMatrixRow(), int(6));
	PTR_CONTROL_PARAMS->setIfNotExist(getKeySizeCtrlMatrixCol(), int(6));

	__parseConfig();
}

CControlMatrixLCSNS::~CControlMatrixLCSNS(void)
{
}

//*********************************************************************************
//FUNCTION:
void CControlMatrixLCSNS::__genControlPointsMatrix( const SamplePointSet& vSamplePointSet, PtrControlPointsSet& voCtrlPntsSet ) const
{
	VecCandCtrlPnts CandCtrlPntsSet;
	__initControlPointsMatrix(vSamplePointSet, CandCtrlPntsSet);
	__removeRedundantPoints(CandCtrlPntsSet, voCtrlPntsSet);

//#ifdef _DEBUG
//	char FileName[255];
//	VecColVector3 TmpCtrlPntSet;
//	for (unsigned int i=0; i<voCtrlPntsSet->size(); ++i)
//	{
//		TmpCtrlPntSet.clear();
//		for (unsigned int s=0; s<m_CtrlMatRow; ++s)
//		{
//			for (unsigned int t=0; t<m_CtrlMatCol; ++t)
//			{
//				TmpCtrlPntSet.push_back((*voCtrlPntsSet)[i].second(s, t));
//			}
//		}
//		sprintf(FileName, "TestData\\output\\CorPointPairs\\FineReg_Sample\\SampleCtrlPnts_%d.ply", i);
//		export2Ply(FileName, TmpCtrlPntSet);
//	}
//#endif
}


//*********************************************************************************
//FUNCTION:
void hiveRegistration::CControlMatrixLCSNS::genControlPointsMatrix(const hivePointCloud::CPointCloud* vPointCloud, PtrControlPointsSet& voCtrlPntsSet)
{
	_ASSERT(vPointCloud);

	if (m_pPointCloud != vPointCloud)
	{
		m_pPointCloud = vPointCloud;

		CBaseSampler* pSample = dynamic_cast<CBaseSampler*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(m_SampleStrID));
		SamplePointSet SampleRes = pSample->samplePointCloud(*vPointCloud);

		__genControlPointsMatrix(SampleRes, voCtrlPntsSet);
	}
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CControlMatrixLCSNS::__computeRegion( const Eigen::Matrix<double, 3, Eigen::Dynamic>& vPointSet, RegionDim3& voRegion ) const
{
	Eigen::Matrix<double, 3, 2> PointsRegion;
	computeRegion<3>(vPointSet.data(), vPointSet.cols(), PointsRegion);

	Eigen::Matrix<double, 3, 1> ExtendLen = (PointsRegion.col(1) - PointsRegion.col(0)) * m_ExtendLengthFactor;

	voRegion <<
		PointsRegion(0, 0) - ExtendLen(0, 0), PointsRegion(0, 1) + ExtendLen(0, 0),
		PointsRegion(1, 0) - ExtendLen(1, 0), PointsRegion(1, 1) + ExtendLen(1, 0), 
		PointsRegion(2, 0) - ExtendLen(2, 0), PointsRegion(2, 1) + ExtendLen(2, 0);
}

//*********************************************************************************
//FUNCTION:
void CControlMatrixLCSNS::__initControlPointsMatrix( const SamplePointSet& vSamplePointSet, VecCandCtrlPnts& voCandCtrlPntsSet ) const
{
	voCandCtrlPntsSet.clear();

	SCtrolMatrixData* pCtlMatData = hiveCommon::CSingleton<SCtrolMatrixData>::getInstance();
	pCtlMatData->RowAndCol.first = m_CtrlMatRow;
	pCtlMatData->RowAndCol.second = m_CtrlMatCol;

	std::vector<unsigned int> NeighbourSet;

	hiveCommon::CKNNSearch KNN;
	KNN.initKNNSearch(m_pPointCloud->getPosPointer(), m_pPointCloud->getNumPoints(), 3, m_NumNeibs);

	const double* pPos = m_pPointCloud->getPosPointer();
	const int SizeExtendRow = m_CtrlMatRow + m_SizeExtendCtrlMat;
	const int SizeExtendCol = m_CtrlMatCol + m_SizeExtendCtrlMat;
	const int HalfSizeExtendRow = m_CtrlMatRow>>1;
	const int HalfSizeExtendCol = m_CtrlMatCol>>1;

	Eigen::Matrix<double, 3, Eigen::Dynamic> AdjustedPointSet;
	AdjustedPointSet.resize(3, m_NumNeibs);

	const VecColVector3& SmpPntSet = vSamplePointSet->getPointSet();
//#ifdef _DEBUG
//	export2Ply("TestData\\output\\CorPointPairs\\FineReg_Sample\\SampleQi.ply", SmpPntSet);
//	char FileName[255];
//	int SampleIndex = 0;
//#endif

	int SizeSample = SmpPntSet.size();

	CUniqueData* pUniqueData = dynamic_cast<CUniqueData*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(CUniqueData::getSigTgtUniqueData()));
	double UnitSquaredDist = pUniqueData->getUniqSquareDist();
	double ZDistThre = m_ZDistThresholdFactor * std::sqrt(UnitSquaredDist);

//#pragma omp parallel omp		//FIXME: is omp safe here?
	for (int i=0; i<SmpPntSet.size(); ++i)
	{
		NeighbourSet.clear();

		KNN.executeKNN(SmpPntSet[i].data(), NeighbourSet);

//#ifdef _DEBUG
//		VecColVector3 TmpVec;
//		TmpVec.push_back(SmpPntSet[i]);
//		sprintf(FileName, "TestData\\output\\CorPointPairs\\FineReg_Sample\\SampleQi_%d.ply", SampleIndex);
//		export2Ply(FileName, TmpVec);
//		sprintf(FileName, "TestData\\output\\CorPointPairs\\FineReg_Sample\\SampleQi_%d_Neibs.ply", SampleIndex);
//		export2Ply(FileName, *m_pPointCloud, NeighbourSet);
//#endif
		
		SCtrolMatrixData::CtrlMatrix TmpCtrlMat;
		ColVector3 CenterPoint;
		ColVector3 Normal;
		ColVector3 NewCoordinateAxisX;
		ColVector3 NewCoordinateAxisY;
		ColVector3 NewCoordinateAxisZ;
		
		__setCoordOrigin(NeighbourSet, CenterPoint, Normal);
		__initNewCoordinates(Normal, NewCoordinateAxisX, NewCoordinateAxisY, NewCoordinateAxisZ);

		TmpCtrlMat.MatWorld2Local << 
			NewCoordinateAxisX(0, 0), NewCoordinateAxisX(1, 0), NewCoordinateAxisX(2, 0),
			NewCoordinateAxisY(0, 0), NewCoordinateAxisY(1, 0), NewCoordinateAxisY(2, 0),
			NewCoordinateAxisZ(0, 0), NewCoordinateAxisZ(1, 0), NewCoordinateAxisZ(2, 0);		// ATTENTION

		for (unsigned int k=0; k<NeighbourSet.size(); ++k)
		{
			ColVector3 Point(pPos[NeighbourSet[k]*3 + 0], pPos[NeighbourSet[k]*3 + 1], pPos[NeighbourSet[k]*3 + 2]);		
			AdjustedPointSet.col(k) = TmpCtrlMat.MatWorld2Local * (Point - CenterPoint);
		}

		RegionDim3 Region;
		__computeRegion(AdjustedPointSet, Region);
		const double UnitLength0 = ( Region(0, 1) - Region(0, 0)) / SizeExtendRow;
		const double UnitLength1 = ( Region(1, 1) - Region(1, 0)) / SizeExtendCol;

		TmpCtrlMat.SizeEachBlock.first  = UnitLength0;
		TmpCtrlMat.SizeEachBlock.second = UnitLength1;
		TmpCtrlMat.Range = Region;
		TmpCtrlMat.Original = CenterPoint;

		pCtlMatData->VecCtrlMatData.push_back(TmpCtrlMat);

//#ifdef _DEBUG
//		VecColVector3 VecAdjustedPointSet;
//		VecColVector3 AcceptedPointSet;
//		VecAdjustedPointSet.reserve(AdjustedPointSet.size());
//		for (unsigned int k=0; k!=AdjustedPointSet.cols(); ++k)
//		{
//			VecAdjustedPointSet.push_back(AdjustedPointSet.col(k));
//		}
//		sprintf(FileName, "TestData\\output\\CorPointPairs\\FineReg_Sample\\SampleQi_%d_Neibs_Adj.ply", SampleIndex);
//		export2Ply(FileName, VecAdjustedPointSet);
//#endif
		
		CandCtrlPnts TmpCtrPnts;
		TmpCtrPnts.resize(m_CtrlMatRow, m_CtrlMatCol);
		for (unsigned int k=0; k!=AdjustedPointSet.cols(); ++k)
		{
			if (AdjustedPointSet(2, 0) < ZDistThre && AdjustedPointSet(2, 0) > (-ZDistThre))
			{
				const int CellIndex0 = (AdjustedPointSet(0, k) - 0) / UnitLength0 + HalfSizeExtendRow;
				const int CellIndex1 = (AdjustedPointSet(1, k) - 0) / UnitLength1 + HalfSizeExtendCol;
				if (CellIndex0 >= 0 && CellIndex0 < m_CtrlMatRow && CellIndex1 >= 0 && CellIndex1 < m_CtrlMatCol)
				{
					TmpCtrPnts(CellIndex0, CellIndex1).push_back(AdjustedPointSet.col(k));

//#ifdef _DEBUG
//					AcceptedPointSet.push_back(AdjustedPointSet.col(k));
//#endif
				}
			}
		}
		voCandCtrlPntsSet.push_back(TmpCtrPnts);

//#ifdef _DEBUG
//		sprintf(FileName, "TestData\\output\\CorPointPairs\\FineReg_Sample\\Accepted_PntSet_%d.ply", SampleIndex);
//		export2Ply(FileName, AcceptedPointSet);
//		SampleIndex++;
//#endif
	}
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CControlMatrixLCSNS::__initNewCoordinates( const ColVector3& vNormal, ColVector3& voNewCoordinateAxisX, ColVector3& voNewCoordinateAxisY, ColVector3& voNewCoordinateAxisZ ) const
{
	_ASSERT(vNormal(0, 0) || vNormal(1, 0) || vNormal(2, 0));

	if ( (0==vNormal(1, 0) && 0==vNormal(2, 0)) )
	{
		voNewCoordinateAxisX << 1, 0, 0;
		voNewCoordinateAxisY << 0, 1, 0;
		voNewCoordinateAxisZ << 0, 0, 1;
	}
	else
	{
		double Denominator = sqrt(vNormal(0, 0)*vNormal(0, 0) + vNormal(1, 0)*vNormal(1, 0) + vNormal(2, 0)*vNormal(2, 0));
		voNewCoordinateAxisZ << vNormal(0, 0)/Denominator, vNormal(1, 0)/Denominator, vNormal(2, 0)/Denominator;
		ColVector3 TempVec(1, 0, 0);
		voNewCoordinateAxisX = voNewCoordinateAxisZ.cross(TempVec);
		voNewCoordinateAxisY = voNewCoordinateAxisX.cross(voNewCoordinateAxisZ);
	}
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CControlMatrixLCSNS::__removeRedundantPoints(VecCandCtrlPnts& vCandCtrlPntsSet, PtrControlPointsSet& voCtrlPntsSet) const
{
	if (!voCtrlPntsSet.get())
	{
		voCtrlPntsSet.reset(new ControlPointsSet);
	}
	voCtrlPntsSet->clear();

	bool ToDeleteCurrentElement = false;

	ControlPoints TmpPntSet;
	TmpPntSet.resize(m_CtrlMatRow, m_CtrlMatCol);

	int SizeCtlPntsSet = vCandCtrlPntsSet.size();

	SCtrolMatrixData* pCtrlMatData = hiveCommon::CSingleton<SCtrolMatrixData>::getInstance();
	_ASSERT(pCtrlMatData);

#ifdef _DEBUG
	VecColVector3 PointSet;
	VecColVector3 CtrlPntsSet;
	VecColVector3 AllPntSet;
	char FileName[255];

	std::string ControlPointSetFile, SurfaceSetFile;
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyControlPointSetFile(), ControlPointSetFile);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeySurfaceSetFile(), SurfaceSetFile);
	_ASSERT(DumpRes);

	std::string CurrentControlPointSetFile, CurrentSurfaceSetFile;

#endif

//#pragma omp parallel for
	for (int s=0; s<SizeCtlPntsSet; ++s)
	{
#ifdef _DEBUG
		Eigen::Matrix3d MatLocal2World = pCtrlMatData->VecCtrlMatData[s].MatWorld2Local.inverse();
		const ColVector3& CoordsOrigin = pCtrlMatData->VecCtrlMatData[s].Original;

		AllPntSet.clear();
#endif

		const std::pair<double, double>& PairSizeBlock = pCtrlMatData->VecCtrlMatData[s].SizeEachBlock;
		const RegionDim3& CurRegion = pCtrlMatData->VecCtrlMatData[s].Range;
		for (int i=0; i<m_CtrlMatRow; ++i)
		{
			for (int k=0; k<m_CtrlMatCol; ++k)
			{
#ifdef _DEBUG
				AllPntSet.insert(AllPntSet.end(), vCandCtrlPntsSet[s](i, k).begin(), vCandCtrlPntsSet[s](i, k).end());
#endif

				unsigned int CurPntSetSize = vCandCtrlPntsSet[s](i, k).size();
				if (CurPntSetSize > 1)
				{
					if (0 == m_MatrixGenerationType)
					{
						int RandomIndex = rand()%CurPntSetSize;		// FIXME: 由于是局部坐标系，范围本来已经比较小，这里可能不再需要随机数
						TmpPntSet(i, k) = vCandCtrlPntsSet[s](i, k)[RandomIndex];
					}
					else
					{
						double SquaredDist = std::numeric_limits<double>::max();
						int IndexMinDist = 0;
						for (unsigned int z=0; z<vCandCtrlPntsSet[s](i, k).size(); ++z)
						{
							double CentPos[2] = { (0.5 + i - (m_CtrlMatRow>>1)) * PairSizeBlock.first, (0.5 + k - (m_CtrlMatCol>>1)) * PairSizeBlock.second };
							double SquareDist2Cent = (vCandCtrlPntsSet[s](i, k)[z](0, 0) - CentPos[0])*(vCandCtrlPntsSet[s](i, k)[z](0, 0) - CentPos[0]) +
								(vCandCtrlPntsSet[s](i, k)[z](1, 0) - CentPos[1])*(vCandCtrlPntsSet[s](i, k)[z](1, 0) - CentPos[1]);
							if ( SquareDist2Cent < SquaredDist )
							{
								SquaredDist = SquareDist2Cent;
								IndexMinDist = z;
							}
						}
						TmpPntSet(i, k) = vCandCtrlPntsSet[s](i, k)[IndexMinDist];
					}
					
				}
				else if (CurPntSetSize < 1)
				{
					ToDeleteCurrentElement = true;
					break;
				}
				else
				{
					TmpPntSet(i, k) = vCandCtrlPntsSet[s](i, k)[0];
				}
			}
			if (ToDeleteCurrentElement)
			{
				break;
			}
		}
		if (ToDeleteCurrentElement)
		{
			ToDeleteCurrentElement = false;
		}
		else
		{
			voCtrlPntsSet->push_back(std::make_pair(s, TmpPntSet));

//#ifdef _DEBUG
//			CtrlPntsSet.clear();
//
//			for (int m=0; m<6; ++m)
//			{
//				for (int n=0; n<6; ++n)
//				{
//					CtrlPntsSet.push_back(MatLocal2World * TmpPntSet(m, n) + CoordsOrigin);
//				}
//			}
//
//			sprintf(FileName, "CtrlPntsSet_%d.ply", s);
//			CurrentControlPointSetFile = ControlPointSetFile + FileName;
//			export2Ply(CurrentControlPointSetFile.c_str(), CtrlPntsSet);
//
//			PointSet.clear();
//			for (int p=0; p!=m_CtrlMatRow-3; ++p)
//			{
//				for (int q=0; q!=m_CtrlMatCol-3; ++q)
//				{
//					CtrlPntsSet.clear();
//					Bicubic::CBicubic BicubicObj;
//					ColVector3 Point;
//					Eigen::Matrix4d	CtrlPoints[3];
//
//					CtrlPoints[0] <<
//						TmpPntSet(p, q  )(0, 0), TmpPntSet(p+1, q  )(0, 0), TmpPntSet(p+2, q  )(0, 0), TmpPntSet(p+3, q  )(0, 0),
//						TmpPntSet(p, q+1)(0, 0), TmpPntSet(p+1, q+1)(0, 0), TmpPntSet(p+2, q+1)(0, 0), TmpPntSet(p+3, q+1)(0, 0),
//						TmpPntSet(p, q+2)(0, 0), TmpPntSet(p+1, q+2)(0, 0), TmpPntSet(p+2, q+2)(0, 0), TmpPntSet(p+3, q+2)(0, 0),
//						TmpPntSet(p, q+3)(0, 0), TmpPntSet(p+1, q+3)(0, 0), TmpPntSet(p+2, q+3)(0, 0), TmpPntSet(p+3, q+3)(0, 0);
//
//					CtrlPoints[1] <<
//						TmpPntSet(p, q  )(1, 0), TmpPntSet(p+1, q  )(1, 0), TmpPntSet(p+2, q  )(1, 0), TmpPntSet(p+3, q  )(1, 0),
//						TmpPntSet(p, q+1)(1, 0), TmpPntSet(p+1, q+1)(1, 0), TmpPntSet(p+2, q+1)(1, 0), TmpPntSet(p+3, q+1)(1, 0),
//						TmpPntSet(p, q+2)(1, 0), TmpPntSet(p+1, q+2)(1, 0), TmpPntSet(p+2, q+2)(1, 0), TmpPntSet(p+3, q+2)(1, 0),
//						TmpPntSet(p, q+3)(1, 0), TmpPntSet(p+1, q+3)(1, 0), TmpPntSet(p+2, q+3)(1, 0), TmpPntSet(p+3, q+3)(1, 0);
//
//					CtrlPoints[2] <<
//						TmpPntSet(p, q  )(2, 0), TmpPntSet(p+1, q  )(2, 0), TmpPntSet(p+2, q  )(2, 0), TmpPntSet(p+3, q  )(2, 0),
//						TmpPntSet(p, q+1)(2, 0), TmpPntSet(p+1, q+1)(2, 0), TmpPntSet(p+2, q+1)(2, 0), TmpPntSet(p+3, q+1)(2, 0),
//						TmpPntSet(p, q+2)(2, 0), TmpPntSet(p+1, q+2)(2, 0), TmpPntSet(p+2, q+2)(2, 0), TmpPntSet(p+3, q+2)(2, 0),
//						TmpPntSet(p, q+3)(2, 0), TmpPntSet(p+1, q+3)(2, 0), TmpPntSet(p+2, q+3)(2, 0), TmpPntSet(p+3, q+3)(2, 0);
//
//					for (double u=0.001; u<1.0; u+=0.025)
//					{
//						for (double v=0.001; v<1.0; v+=0.025)
//						{
//							BicubicObj.compute3DPointPosByUV(u, v, CtrlPoints, Point);
//							PointSet.push_back(MatLocal2World * Point + CoordsOrigin);
//						}
//					}
//				}
//			}
//			sprintf(FileName, "SurfPns_%d.ply", s);
//			CurrentSurfaceSetFile = SurfaceSetFile + FileName;
//			export2Ply(CurrentSurfaceSetFile.c_str(), PointSet);
//#endif

		}
	}
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CControlMatrixLCSNS::__parseConfig( void )
{
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeySizeCtrlMatrixRow(), m_CtrlMatRow);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeySizeCtrlMatrixCol(), m_CtrlMatCol);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyNumNeibs(), m_NumNeibs);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeySizeExtendCtrlMat(), m_SizeExtendCtrlMat);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyExtendLengthFactor(), m_ExtendLengthFactor);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeySampleStrID(), m_SampleStrID);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyMatrixGenerationType(), m_MatrixGenerationType);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyZDistThreshodlFactor(), m_ZDistThresholdFactor);
	_ASSERT(DumpRes);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CControlMatrixLCSNS::__setCoordOrigin( const std::vector<unsigned int>& vPointSet, ColVector3& voOrigin, ColVector3& voNormal ) const
{
	double XYZ[3] = {0, 0, 0};
	const double* pPos = m_pPointCloud->getPosPointer();
	const double* pNormal = m_pPointCloud->getNormalPointer();
	Eigen::Matrix<double, 3, Eigen::Dynamic> NeibPoints;
	NeibPoints.resize(3, vPointSet.size());

	int Size = vPointSet.size();

#pragma omp parallel for
	for (int i=0; i<Size; ++i)
	{
		XYZ[0] += pPos[vPointSet[i] * 3];
		XYZ[1] += pPos[vPointSet[i] * 3 + 1];
		XYZ[2] += pPos[vPointSet[i] * 3 + 2];
		NeibPoints(0, i) = pPos[vPointSet[i] * 3];
		NeibPoints(1, i) = pPos[vPointSet[i] * 3 + 1];
		NeibPoints(2, i) = pPos[vPointSet[i] * 3 + 2];
	}

	XYZ[0] /= vPointSet.size();
	XYZ[1] /= vPointSet.size();
	XYZ[2] /= vPointSet.size();

	hiveCommon::CKNNSearch KNN;
	KNN.initKNNSearch(NeibPoints.data(), Size, 3, 1);
	std::vector<unsigned int> Neibs;
	KNN.executeKNN(XYZ, Neibs);

	unsigned int PointCloudIndex = vPointSet[Neibs.front()];
	voOrigin << NeibPoints.col(Neibs.front());
	voNormal << pNormal[PointCloudIndex*3], pNormal[PointCloudIndex*3 + 1], pNormal[PointCloudIndex*3 + 2];
}