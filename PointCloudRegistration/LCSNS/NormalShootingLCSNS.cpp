#include "NormalShootingLCSNS.h"
#include <cmath>
#include <Eigen/LU>
#include "ICPMacros.h"
#include "ControlParameters.h"
#include "ProductFactoryData.h"
#include "KNNSearch.h"
#include "CommonLCSNSSubset.hpp"
#include "ControlMatrixLCSNS.h"
#include "Bicubic.h"
#include "Intersection.h"
#include "LCSNSType.h"
#include "ControlMatrixData.h"
#include "UniqueData.h"

 //#ifdef _DEBUG
 #include "TestUnitity.h"
 //#endif

using namespace hiveRegistration;

CNormalShootingLCSNS::CNormalShootingLCSNS()
: m_pKNN(new hiveCommon::CKNNSearch)
{
	__parseConfig();
}

//*********************************************************************************
//FUNCTION:
void CNormalShootingLCSNS::determineCorrespondencePairSet(
	const CorrespondencePointSet& vTgtCorPointSet, const hivePointCloud::CPointCloud* vSourcePointCloud, 
	const Eigen::Matrix3d& vR, const ColVector3& vT, 
	CorrespondencePairSet& voCorPairSet )
{
	_ASSERT(vTgtCorPointSet.get() && vSourcePointCloud);

	const CCommonLCSNSSubset<PtrControlPointsSet>* pCorPointSet = dynamic_cast<const CCommonLCSNSSubset<PtrControlPointsSet>*>(vTgtCorPointSet.get());
	_ASSERT(pCorPointSet);

	const VecColVector3& CorPntSet = pCorPointSet->getPointSet();
	const VecColVector3& CorNormSet = pCorPointSet->getNormalSet();
	const PtrControlPointsSet& CorCtrlPntsSet = pCorPointSet->getControlPointsSet();
	const std::vector<std::pair<int, int> >& CorSurfIdxSet = pCorPointSet->getIndexPairSet();
	const std::vector<std::pair<double, double> >& CorUVSet = pCorPointSet->getUVSet();

	const SCtrolMatrixData* pCtlMatData = hiveCommon::CSingleton<SCtrolMatrixData>::getInstance();

	CUniqueData* pUniqueData = dynamic_cast<CUniqueData*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(CUniqueData::getSigTgtUniqueData()));
	double UnitSquareDist = pUniqueData->getUniqSquareDist();
	double Dist2Move = sqrt(UnitSquareDist) * m_LenFact2Move;

	ControlPoints TmpSrcCtrlPnts;
	PtrControlPointsSet TgtCtrlPntsSet(new ControlPointsSet);
	PtrControlPointsSet SrcCtrlPntsSet(new ControlPointsSet);
	std::vector<std::pair<int, int> > TgtIndexPairSet;
	std::vector<std::pair<int, int> > SrcIndexPairSet;
	std::vector<std::pair<double, double> > TgtUVSet;
	std::vector<std::pair<double, double> > SrcUVSet;

	hivePointCloud::CPointCloud TransformPointCloud = *vSourcePointCloud;
	TransformPointCloud.transform(vR.data(), vT.data());
	m_pKNN->initKNNSearch(TransformPointCloud.getPosPointer(), TransformPointCloud.getNumPoints(), 3, m_NumNeibP);

	int SizeCorPntSet = CorPntSet.size();

// #ifdef _DEBUG
// 	int IntCount = 0;
// #endif

//#pragma parallel for
	for (int i=0; i<SizeCorPntSet; ++i)
	{
		bool IsUseful = false;
		int SampleIdx = (*CorCtrlPntsSet)[i].first;

		__computeControlPointsByMatrix(TransformPointCloud, SampleIdx, TmpSrcCtrlPnts, IsUseful);
		
		if ( IsUseful )
		{
			const ColVector3& RayDir = CorNormSet[i];
			ColVector3 RayOrigin = CorPntSet[i] - RayDir * Dist2Move;
			Bicubic::CBicubic BicubicObj;

			std::pair<int, int> IntersectedSurfIdx;
			ColVector2 IntersectedUV;
			bool IsIntersected = false;
			const unsigned int CtrlMatSpan = 4;
			SRegionIndexRange RegionIndexRange = { 0, pCtlMatData->RowAndCol.first - CtrlMatSpan, 0, pCtlMatData->RowAndCol.second - CtrlMatSpan };

#ifdef _DEBUG
			VecColVector3 SrcCtrlPntsSetTTT, TgtCtrlPntsSetTTT;
			VecColVector3 SP, SP1;
			const ColVector3& CenterPoint = pCtlMatData->VecCtrlMatData[SampleIdx].Original;
			const Eigen::Matrix3d& CurMatWorld2Local = pCtlMatData->VecCtrlMatData[SampleIdx].MatWorld2Local;

			for (int s=0; s<TmpSrcCtrlPnts.rows(); ++s)
			{
				for (int t=0; t<TmpSrcCtrlPnts.cols(); ++t)
				{
					SrcCtrlPntsSetTTT.push_back(CurMatWorld2Local.inverse()*TmpSrcCtrlPnts(s, t) + CenterPoint);
					TgtCtrlPntsSetTTT.push_back(CurMatWorld2Local.inverse()*(*CorCtrlPntsSet)[i].second(s, t) + CenterPoint);
					//SP.push_back(RayOrigin);
					//SP1.push_back(CorPntSet[i] + RayDir * Dist2Move);
				}
			}
			char FileName[255];
			sprintf(FileName, "TestData\\output\\CorPointPairs\\FineReg_Mid\\CtrlPntsSet\\Src_Ctrl_Pnts_Set_%d.ply", i);
			export2Ply(FileName, SrcCtrlPntsSetTTT);
			sprintf(FileName, "TestData\\output\\CorPointPairs\\FineReg_Mid\\CtrlPntsSet\\Tgt_Ctrl_Pnts_Set_%d.ply", i);
			export2Ply(FileName, TgtCtrlPntsSetTTT);
			//export2Ply("TestData\\output\\CorPointPairs\\FineReg_Mid\\CtrlPntsSet\\Src_Pnt.ply", SP);
			//export2Ply("TestData\\output\\CorPointPairs\\FineReg_Mid\\CtrlPntsSet\\Src_Pnt1.ply", SP1);
#endif

			__computeIntersectPointWithEnoughSurfacePatchs(RegionIndexRange, RayOrigin, RayDir, TmpSrcCtrlPnts, IntersectedSurfIdx, IntersectedUV, IsIntersected);

			if (IsIntersected)
			{
				SrcCtrlPntsSet->push_back(std::make_pair(SampleIdx, TmpSrcCtrlPnts));
				TgtCtrlPntsSet->push_back((*CorCtrlPntsSet)[i]);
				SrcIndexPairSet.push_back(IntersectedSurfIdx);
				TgtIndexPairSet.push_back(CorSurfIdxSet[i]);
				SrcUVSet.push_back(std::make_pair(IntersectedUV(0, 0), IntersectedUV(1, 0)));
				TgtUVSet.push_back(CorUVSet[i]);

// #ifdef _DEBUG
// 				++IntCount;
// #endif
			}
		}
		else
		{
			continue;
		}
	}

	voCorPairSet.first = makeSharedPtr(new CCommonLCSNSSubset<PtrControlPointsSet>(SrcCtrlPntsSet));
	voCorPairSet.second = makeSharedPtr(new CCommonLCSNSSubset<PtrControlPointsSet>(TgtCtrlPntsSet));
	CCommonLCSNSSubset<PtrControlPointsSet>* pSrcCor = dynamic_cast<CCommonLCSNSSubset<PtrControlPointsSet>*>((voCorPairSet.first).get());
	CCommonLCSNSSubset<PtrControlPointsSet>* pTgtCor = dynamic_cast<CCommonLCSNSSubset<PtrControlPointsSet>*>((voCorPairSet.second).get());
	pSrcCor->swapUVSet(SrcIndexPairSet, SrcUVSet);
	pTgtCor->swapUVSet(TgtIndexPairSet, TgtUVSet);
}

//*********************************************************************************
//FUNCTION:
void CNormalShootingLCSNS::__computeIntersectPointWithEnoughSurfacePatchs(
	const SRegionIndexRange& vIndexRange, const ColVector3& vRayOrigin, const ColVector3& vDir, const ControlPoints& vControlPoints,
	std::pair<int, int>& voIntersectedSurfIdx, ColVector2& voUV, bool& voIsIntersected) const
{
	Eigen::Matrix4d	CtrlPoints[3];
	
	voIsIntersected = false;

	for (int i=vIndexRange.RowBegin; i<=vIndexRange.RowEnd; ++i)
	{
		for (int k=vIndexRange.ColBegin; k<=vIndexRange.ColEnd; ++k)
		{
			CtrlPoints[0] <<
				vControlPoints(i, k  )(0, 0), vControlPoints(i+1, k  )(0, 0), vControlPoints(i+2, k  )(0, 0), vControlPoints(i+3, k  )(0, 0),
				vControlPoints(i, k+1)(0, 0), vControlPoints(i+1, k+1)(0, 0), vControlPoints(i+2, k+1)(0, 0), vControlPoints(i+3, k+1)(0, 0),
				vControlPoints(i, k+2)(0, 0), vControlPoints(i+1, k+2)(0, 0), vControlPoints(i+2, k+2)(0, 0), vControlPoints(i+3, k+2)(0, 0),
				vControlPoints(i, k+3)(0, 0), vControlPoints(i+1, k+3)(0, 0), vControlPoints(i+2, k+3)(0, 0), vControlPoints(i+3, k+3)(0, 0);

			CtrlPoints[1] <<
				vControlPoints(i, k  )(1, 0), vControlPoints(i+1, k  )(1, 0), vControlPoints(i+2, k  )(1, 0), vControlPoints(i+3, k  )(1, 0),
				vControlPoints(i, k+1)(1, 0), vControlPoints(i+1, k+1)(1, 0), vControlPoints(i+2, k+1)(1, 0), vControlPoints(i+3, k+1)(1, 0),
				vControlPoints(i, k+2)(1, 0), vControlPoints(i+1, k+2)(1, 0), vControlPoints(i+2, k+2)(1, 0), vControlPoints(i+3, k+2)(1, 0),
				vControlPoints(i, k+3)(1, 0), vControlPoints(i+1, k+3)(1, 0), vControlPoints(i+2, k+3)(1, 0), vControlPoints(i+3, k+3)(1, 0);

			CtrlPoints[2] <<
				vControlPoints(i, k  )(2, 0), vControlPoints(i+1, k  )(2, 0), vControlPoints(i+2, k  )(2, 0), vControlPoints(i+3, k  )(2, 0),
				vControlPoints(i, k+1)(2, 0), vControlPoints(i+1, k+1)(2, 0), vControlPoints(i+2, k+1)(2, 0), vControlPoints(i+3, k+1)(2, 0),
				vControlPoints(i, k+2)(2, 0), vControlPoints(i+1, k+2)(2, 0), vControlPoints(i+2, k+2)(2, 0), vControlPoints(i+3, k+2)(2, 0),
				vControlPoints(i, k+3)(2, 0), vControlPoints(i+1, k+3)(2, 0), vControlPoints(i+2, k+3)(2, 0), vControlPoints(i+3, k+3)(2, 0);

			voIsIntersected = intersect(CtrlPoints, vRayOrigin, vDir, voUV);

			if (voIsIntersected)
			{
				voIntersectedSurfIdx.first = i;
				voIntersectedSurfIdx.second = k;
				return;
			}
		}
	}
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CNormalShootingLCSNS::__computeControlPointsByMatrix( 
	const hivePointCloud::CPointCloud& vTransformedPointCloud, int vSampleIdx,
	ControlPoints& voSrcCtrlPnts, bool& voIsUseful ) const
{
	const SCtrolMatrixData* pCtlMatData = hiveCommon::CSingleton<SCtrolMatrixData>::getInstance();
	const ColVector3& CenterPoint = pCtlMatData->VecCtrlMatData[vSampleIdx].Original;
	std::vector<unsigned> Neibs;
	m_pKNN->executeKNN(CenterPoint.data(), Neibs);

	Eigen::Matrix<double, 3, Eigen::Dynamic> NeibPoints;
	NeibPoints.resize(3, Neibs.size());

	const double* pPos = vTransformedPointCloud.getPosPointer();

	unsigned int i = 0;
	for (std::vector<unsigned>::const_iterator CItr=Neibs.begin(); CItr!=Neibs.end(); ++CItr, ++i)
	{
		NeibPoints(0, i) = pPos[(*CItr)*3 + 0];
		NeibPoints(1, i) = pPos[(*CItr)*3 + 1];
		NeibPoints(2, i) = pPos[(*CItr)*3 + 2];
	}
	
	__genControlPointsMatrix(NeibPoints, vSampleIdx, voSrcCtrlPnts, voIsUseful);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CNormalShootingLCSNS::__genControlPointsMatrix( 
	const Eigen::Matrix<double, 3, Eigen::Dynamic>& vPoints, int vSampleIdx, 
	ControlPoints& voSrcCtrlPnts, bool& voIsUseful ) const
{
	CandCtrlPnts TmpCandCtrlPnts;
	__initControlPoints(vPoints, vSampleIdx, TmpCandCtrlPnts);
	__removeRedundantPoints(TmpCandCtrlPnts, vSampleIdx, voSrcCtrlPnts, voIsUseful);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CNormalShootingLCSNS::__removeRedundantPoints( const CandCtrlPnts& vCandCtrlPnts, int vSampleIdx, ControlPoints& voSrcCtrlPnts, bool& voIsUseful ) const
{
	voIsUseful = true;
	const int SizeCtrlMatrixRow = vCandCtrlPnts.rows();
	const int SizeCtrlMatrixCol = vCandCtrlPnts.cols();
	voSrcCtrlPnts.resize(SizeCtrlMatrixRow, SizeCtrlMatrixCol);

	SCtrolMatrixData* pCtrlMatData = hiveCommon::CSingleton<SCtrolMatrixData>::getInstance();
	_ASSERT(pCtrlMatData);

	const std::pair<double, double>& PairSizeBlock = pCtrlMatData->VecCtrlMatData[vSampleIdx].SizeEachBlock;
	const RegionDim3& CurRegion = pCtrlMatData->VecCtrlMatData[vSampleIdx].Range;

	for (int i=0; i<SizeCtrlMatrixRow; ++i)
	{
		for (int k=0; k<SizeCtrlMatrixCol; ++k)
		{
			unsigned int CurPntSetSize = vCandCtrlPnts(i, k).size();
			if (CurPntSetSize > 1)
			{
				if (0 == m_MatrixGenerationType)
				{
					int RandomIndex = rand()%CurPntSetSize;		// FIXME: 随机和不随机效果可能差不多
					voSrcCtrlPnts(i, k) = vCandCtrlPnts(i, k)[RandomIndex];
				} 
				else
				{
					double SquaredDist = std::numeric_limits<double>::max();
					int IndexMinDist = 0;
					for (unsigned int z=0; z<vCandCtrlPnts(i, k).size(); ++z)
					{
						double CentPos[2] = { (0.5 + i - (SizeCtrlMatrixRow>>1)) * PairSizeBlock.first, (0.5 + k - (SizeCtrlMatrixCol>>1)) * PairSizeBlock.second };
						double SquareDist2Cent = (vCandCtrlPnts(i, k)[z](0, 0) - CentPos[0])*(vCandCtrlPnts(i, k)[z](0, 0) - CentPos[0]) +
							(vCandCtrlPnts(i, k)[z](1, 0) - CentPos[1])*(vCandCtrlPnts(i, k)[z](1, 0) - CentPos[1]);
						if ( SquareDist2Cent < SquaredDist )
						{
							SquaredDist = SquareDist2Cent;
							IndexMinDist = z;
						}
					}
					voSrcCtrlPnts(i, k) = vCandCtrlPnts(i, k)[IndexMinDist];
				}
			}
			else if (CurPntSetSize < 1)
			{
				voIsUseful = false;
				return;
			}
			else
			{
				voSrcCtrlPnts(i, k) = vCandCtrlPnts(i, k)[0];
			}
		}
	}
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CNormalShootingLCSNS::__initControlPoints(
	const Eigen::Matrix<double, 3, Eigen::Dynamic>& vPoints, int vSampleIdx,
	CandCtrlPnts& voControlPoints ) const
{
	const SCtrolMatrixData* pCtlMatData = hiveCommon::CSingleton<SCtrolMatrixData>::getInstance();
	register double UnitLength0 = pCtlMatData->VecCtrlMatData[vSampleIdx].SizeEachBlock.first;
	register double UnitLength1 = pCtlMatData->VecCtrlMatData[vSampleIdx].SizeEachBlock.second;

	int MatRow = pCtlMatData->RowAndCol.first;
	int MatCol = pCtlMatData->RowAndCol.second;
	int HalfSizeExtendRow = MatRow * 0.5;
	int HalfSizeExtendCol = MatCol * 0.5;

	const ColVector3& CenterPoint = pCtlMatData->VecCtrlMatData[vSampleIdx].Original;
	const Eigen::Matrix3d& CurMatWorld2Local = pCtlMatData->VecCtrlMatData[vSampleIdx].MatWorld2Local;
	
	voControlPoints.resize(MatRow, MatCol);

	const int NumPoint = vPoints.cols();

// #ifdef _DEBUG
// 	VecColVector3 TmpCtrlPnts;
// 	VecColVector3 TmpOriginalPnts;
// 	char FileName[255];
// #endif

	CUniqueData* pUniqueData = dynamic_cast<CUniqueData*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(CUniqueData::getSigTgtUniqueData()));
	double UnitSuqredDist = pUniqueData->getUniqSquareDist();
	double ZDistThre = m_ZDistThresholdFactor * std::sqrt(UnitSuqredDist);

	//#pragma omp parallel for
	for (int i=0; i<NumPoint; ++i)
	{
		const ColVector3& Point = vPoints.block(0, i, 3, 1);
		ColVector3 NewPos = CurMatWorld2Local * (Point - CenterPoint);

		if (NewPos(2, 0) > (-ZDistThre) && NewPos(2, 0) < ZDistThre)
		{
			const int CellIndex0 = (NewPos(0, 0) - 0) / UnitLength0 + HalfSizeExtendRow;
			const int CellIndex1 = (NewPos(1, 0) - 0) / UnitLength1 + HalfSizeExtendCol;
			if (CellIndex0 >= 0 && CellIndex0 < MatRow && CellIndex1 >= 0 && CellIndex1 < MatCol)
			{
				voControlPoints(CellIndex0, CellIndex1).push_back(NewPos);

// #ifdef _DEBUG
// 				TmpCtrlPnts.push_back(NewPos);
// 				TmpOriginalPnts.push_back(Point);
// #endif
			}
		}
	}

// #ifdef _DEBUG
// 	sprintf(FileName, "TestData\\output\\CorPointPairs\\FineReg_Mid\\Pi\\Init_Ctrl_Pnts_%d.ply", vSampleIdx);
// 	export2Ply(FileName, TmpCtrlPnts);
// 	sprintf(FileName, "TestData\\output\\CorPointPairs\\FineReg_Mid\\Pi\\Init_Ctrl_Pnts_Original_%d.ply", vSampleIdx);
// 	export2Ply(FileName, TmpOriginalPnts);
// #endif
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CNormalShootingLCSNS::determineCorrespondencePairSet( 
	const CorrespondencePointSet& vSrcCorPointSet, const CorrespondencePointSet &vTgtCorPointSet, 
	CorrespondencePairSet& voCorPairSet, std::vector<unsigned int>& voIntersectedPointSet )
{
	_ASSERT(vSrcCorPointSet.get() && vTgtCorPointSet.get());

	voIntersectedPointSet.clear();

	const CCommonLCSNSSubset<PtrControlPointsSet>* pSrcCorPointSet = dynamic_cast<const CCommonLCSNSSubset<PtrControlPointsSet>*>(vSrcCorPointSet.get());
	_ASSERT(pSrcCorPointSet);
	const CCommonLCSNSSubset<PtrControlPointsSet>* pTgtCorPointSet = dynamic_cast<const CCommonLCSNSSubset<PtrControlPointsSet>*>(vTgtCorPointSet.get());
	_ASSERT(pTgtCorPointSet);

	const SCtrolMatrixData* pCtlMatData = hiveCommon::CSingleton<SCtrolMatrixData>::getInstance();

	const PtrControlPointsSet& SrcCorCtrlPntsSet = pSrcCorPointSet->getControlPointsSet();
	const PtrControlPointsSet& TgtCorCtrlPntsSet = pTgtCorPointSet->getControlPointsSet();
	const VecColVector3& SrcCorPointSet = pSrcCorPointSet->getPointSet();
	const VecColVector3& SrcCorNormalSet = pSrcCorPointSet->getNormalSet();
	const std::vector<std::pair<int, int> >& SrcCorSurfIdxSet = pSrcCorPointSet->getIndexPairSet();
	const std::vector<std::pair<double, double> >& SrcCorUVSet = pSrcCorPointSet->getUVSet();

	PtrControlPointsSet SrcCtrlPntsSet(new ControlPointsSet);
	PtrControlPointsSet TgtCtrlPntsSet(new ControlPointsSet);
	std::vector<std::pair<int, int> > SrcSurIdxSet;
	std::vector<std::pair<int, int> > TgtSurfIdxSet;
	std::vector<std::pair<double, double> > SrcUVSet;
	std::vector<std::pair<double, double> > TgtUVSet;

	for (unsigned int i=0; i!=SrcCorPointSet.size(); ++i)
	{
		const ControlPoints& CurTgtCtrlPnts = (*TgtCorCtrlPntsSet)[i].second;

		std::pair<int, int> IntersectedSurfIdx;
		bool IsIntersected = false;
		const unsigned int CtrlMatSpan = 4;
		SRegionIndexRange RegionIndexRange = {0, pCtlMatData->RowAndCol.first - CtrlMatSpan, 0, pCtlMatData->RowAndCol.second - CtrlMatSpan};
		ColVector2 IntersectedUV;

		__computeIntersectPointWithEnoughSurfacePatchs(RegionIndexRange, SrcCorPointSet[i], SrcCorNormalSet[i], CurTgtCtrlPnts, IntersectedSurfIdx, IntersectedUV, IsIntersected);
		
		if (IsIntersected)
		{
			SrcCtrlPntsSet->push_back((*SrcCorCtrlPntsSet)[i]);
			TgtCtrlPntsSet->push_back((*TgtCorCtrlPntsSet)[i]);
			SrcSurIdxSet.push_back(SrcCorSurfIdxSet[i]);
			TgtSurfIdxSet.push_back(IntersectedSurfIdx);
			SrcUVSet.push_back(SrcCorUVSet[i]);
			TgtUVSet.push_back(std::make_pair(IntersectedUV(0, 0), IntersectedUV(1, 0)));

			voIntersectedPointSet.push_back(i);
		}
	}
	voCorPairSet.first = makeSharedPtr(new CCommonLCSNSSubset<PtrControlPointsSet>(SrcCtrlPntsSet));
	voCorPairSet.second = makeSharedPtr(new CCommonLCSNSSubset<PtrControlPointsSet>(TgtCtrlPntsSet));
	CCommonLCSNSSubset<PtrControlPointsSet>* pPairedCorSrcPntSet = dynamic_cast<CCommonLCSNSSubset<PtrControlPointsSet>*>((voCorPairSet.first).get());
	CCommonLCSNSSubset<PtrControlPointsSet>* pPairedCorTgtPntSet = dynamic_cast<CCommonLCSNSSubset<PtrControlPointsSet>*>((voCorPairSet.second).get());
	pPairedCorSrcPntSet->swapUVSet(SrcSurIdxSet, SrcUVSet);
	pPairedCorTgtPntSet->swapUVSet(TgtSurfIdxSet, TgtUVSet);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CNormalShootingLCSNS::__parseConfig( void )
{
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(CControlMatrixLCSNS::getKeySizeExtendCtrlMat(), m_SizeExtendMat);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyNumNeibP(), m_NumNeibP);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyLenFact2Move(), m_LenFact2Move);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(CControlMatrixLCSNS::getKeyZDistThreshodlFactor(), m_ZDistThresholdFactor);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(CControlMatrixLCSNS::getKeyMatrixGenerationType(), m_MatrixGenerationType);
	_ASSERT(DumpRes);
}
