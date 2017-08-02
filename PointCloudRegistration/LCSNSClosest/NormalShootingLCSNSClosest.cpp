#include "NormalShootingLCSNSClosest.h"
#include <cmath>
#include <Eigen/LU>
#include "ICPMacros.h"
#include "ControlParameters.h"
#include "ProductFactoryData.h"
#include "KNNSearch.h"
#include "CommonLCSNSSubset.hpp"
#include "ControlMatrixLCSNSClosest.h"
#include "Bicubic.h"
#include "Intersection.h"
#include "LCSNSType.h"
#include "ControlMatrixData.h"
#include "UniqueData.h"

 //#ifdef _DEBUG
 #include "TestUnitity.h"
 //#endif

using namespace hiveRegistration;

CNormalShootingLCSNSClosest::CNormalShootingLCSNSClosest()
: m_pKNN(new hiveCommon::CKNNSearch)
{
	__parseConfig();
}

//*********************************************************************************
//FUNCTION:
void CNormalShootingLCSNSClosest::determineCorrespondencePairSet(
	const CorrespondencePointSet& vSrcCorPntSet, 
	const Eigen::Matrix3d& vR, const ColVector3& vT, 
	CorrespondencePairSet& voCorPairSet )
{
	_ASSERT(vSrcCorPntSet.get());

	const CCommonLCSNSSubset<PtrSimpleCtrlPntSet>* pSrcCorPntSet = dynamic_cast<const CCommonLCSNSSubset<PtrSimpleCtrlPntSet>*>(vSrcCorPntSet.get());
	_ASSERT(pSrcCorPntSet);

	const VecColVector3& CorPntSet = pSrcCorPntSet->getPointSet();
	const VecColVector3& CorNormSet = pSrcCorPntSet->getNormalSet();
	const PtrSimpleCtrlPntSet& CorCtrlPntsSet = pSrcCorPntSet->getControlPointsSet();
	const std::vector<std::pair<int, int> >& CorSurfIdxSet = pSrcCorPntSet->getIndexPairSet();
	const std::vector<std::pair<double, double> >& CorUVSet = pSrcCorPntSet->getUVSet();

	CUniqueData* pUniqueData = dynamic_cast<CUniqueData*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(CUniqueData::getSigTgtUniqueData()));
	double UnitSquareDist = pUniqueData->getUniqSquareDist();
	double Dist2Move = sqrt(UnitSquareDist) * m_LenFact2Move;
	double CtrlPntsClosestDist = UnitSquareDist * m_ClosestDistFactor;

	std::vector<unsigned int> TmpTgtNeibs;
	ControlPoints TmpSrcCtrlPnts;
	ControlPoints TmpTgtCtrlPnts;
	PtrSimpleCtrlPntSet SrcCtrlPntsSet(new SimpleCtrlPntSet);
	PtrSimpleCtrlPntSet TgtCtrlPntsSet(new SimpleCtrlPntSet);
	std::vector<std::pair<int, int> > SrcIndexPairSet;
	std::vector<std::pair<int, int> > TgtIndexPairSet;
	std::vector<std::pair<double, double> > SrcUVSet;
	std::vector<std::pair<double, double> > TgtUVSet;

	const double* pTgtPos = m_pTgtPntCld->getPosPointer();
	TmpSrcCtrlPnts.resize(m_MatRows, m_MatCols);
	TmpTgtCtrlPnts.resize(m_MatRows, m_MatCols);

	int SizeCorPntSet = CorPntSet.size();

// #ifdef _DEBUG
// 	int IntCount = 0;
// #endif

//#pragma parallel for
	for (int i=0; i<SizeCorPntSet; ++i)
	{
		bool IsUseful = true;
		const ControlPoints& TmpCtrlPntMat = (*CorCtrlPntsSet)[i];
		
		for (int t=0; t<TmpCtrlPntMat.cols(); ++t)
		{
			if (!IsUseful) break;

			for (int s=0; s<TmpCtrlPntMat.rows(); ++s)
			{
				TmpTgtNeibs.clear();
				TmpSrcCtrlPnts(s, t) = vR * TmpCtrlPntMat(s, t) + vT;
				m_pKNN->executeKNN(TmpSrcCtrlPnts(s, t).data(), TmpTgtNeibs);
				double TmpSquaredDist = comSquaredDist(pTgtPos, TmpTgtNeibs.front(), (const double*)TmpSrcCtrlPnts(s, t).data(), 0);
				if (TmpSquaredDist < CtrlPntsClosestDist)
				{
					TmpTgtCtrlPnts(s, t)(0, 0) = pTgtPos[TmpTgtNeibs.front()*3 + 0];
					TmpTgtCtrlPnts(s, t)(1, 0) = pTgtPos[TmpTgtNeibs.front()*3 + 1];
					TmpTgtCtrlPnts(s, t)(2, 0) = pTgtPos[TmpTgtNeibs.front()*3 + 2];

					IsUseful = true;
				}
				else
				{
					IsUseful = false;
					break;
				}
			}
		}
		
		if ( IsUseful )
		{
			const ColVector3& RayDir = CorNormSet[i];
			ColVector3 RayOrigin = CorPntSet[i] - RayDir * Dist2Move;
			Bicubic::CBicubic BicubicObj;

			std::pair<int, int> IntersectedSurfIdx;
			ColVector2 IntersectedUV;
			bool IsIntersected = false;
			const unsigned int CtrlMatSpan = 4;
			SRegionIndexRange RegionIndexRange = { 0, m_MatRows - CtrlMatSpan, 0, m_MatCols - CtrlMatSpan };

#ifdef _DEBUG
			VecColVector3 SrcCtrlPntsSetTTT, TgtCtrlPntsSetTTT;
			VecColVector3 SP, SP1;

			for (int s=0; s<TmpTgtCtrlPnts.rows(); ++s)
			{
				for (int t=0; t<TmpTgtCtrlPnts.cols(); ++t)
				{
					SrcCtrlPntsSetTTT.push_back(TmpTgtCtrlPnts(s, t));
					TgtCtrlPntsSetTTT.push_back(TmpSrcCtrlPnts(s, t));
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

			__computeIntersectPointWithEnoughSurfacePatchs(RegionIndexRange, RayOrigin, RayDir, TmpTgtCtrlPnts, IntersectedSurfIdx, IntersectedUV, IsIntersected);

			if (IsIntersected)
			{
				SrcCtrlPntsSet->push_back(TmpSrcCtrlPnts);
				TgtCtrlPntsSet->push_back(TmpTgtCtrlPnts);
				SrcIndexPairSet.push_back(CorSurfIdxSet[i]);
				TgtIndexPairSet.push_back(IntersectedSurfIdx);
				SrcUVSet.push_back(CorUVSet[i]);
				TgtUVSet.push_back(std::make_pair(IntersectedUV(0, 0), IntersectedUV(1, 0)));

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

	voCorPairSet.first = makeSharedPtr(new CCommonLCSNSSubset<PtrSimpleCtrlPntSet>(SrcCtrlPntsSet));
	voCorPairSet.second = makeSharedPtr(new CCommonLCSNSSubset<PtrSimpleCtrlPntSet>(TgtCtrlPntsSet));
	CCommonLCSNSSubset<PtrSimpleCtrlPntSet>* pSrcCor = dynamic_cast<CCommonLCSNSSubset<PtrSimpleCtrlPntSet>*>((voCorPairSet.first).get());
	CCommonLCSNSSubset<PtrSimpleCtrlPntSet>* pTgtCor = dynamic_cast<CCommonLCSNSSubset<PtrSimpleCtrlPntSet>*>((voCorPairSet.second).get());
	pSrcCor->swapUVSet(SrcIndexPairSet, SrcUVSet);
	pTgtCor->swapUVSet(TgtIndexPairSet, TgtUVSet);
}

//*********************************************************************************
//FUNCTION:
void CNormalShootingLCSNSClosest::__computeIntersectPointWithEnoughSurfacePatchs(
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
void hiveRegistration::CNormalShootingLCSNSClosest::determineCorrespondencePairSet( 
	const CorrespondencePointSet& vSrcCorPntSet, const CorrespondencePointSet &vTgtCorPntSet, 
	CorrespondencePairSet& voCorPairSet, std::vector<unsigned int>& voIntersectedPointSet )
{
	_ASSERT(vSrcCorPntSet.get() && vTgtCorPntSet.get());

	voIntersectedPointSet.clear();

	const CCommonLCSNSSubset<PtrSimpleCtrlPntSet>* pSrcCorPointSet = dynamic_cast<const CCommonLCSNSSubset<PtrSimpleCtrlPntSet>*>(vSrcCorPntSet.get());
	_ASSERT(pSrcCorPointSet);
	const CCommonLCSNSSubset<PtrSimpleCtrlPntSet>* pTgtCorPointSet = dynamic_cast<const CCommonLCSNSSubset<PtrSimpleCtrlPntSet>*>(vTgtCorPntSet.get());
	_ASSERT(pTgtCorPointSet);

	const PtrSimpleCtrlPntSet& SrcCorCtrlPntsSet = pSrcCorPointSet->getControlPointsSet();
	const PtrSimpleCtrlPntSet& TgtCorCtrlPntsSet = pTgtCorPointSet->getControlPointsSet();
	const VecColVector3& SrcCorPointSet = pSrcCorPointSet->getPointSet();
	const VecColVector3& SrcCorNormalSet = pSrcCorPointSet->getNormalSet();
	const std::vector<std::pair<int, int> >& SrcCorSurfIdxSet = pSrcCorPointSet->getIndexPairSet();
	const std::vector<std::pair<double, double> >& SrcCorUVSet = pSrcCorPointSet->getUVSet();

	PtrSimpleCtrlPntSet SrcCtrlPntsSet(new SimpleCtrlPntSet);
	PtrSimpleCtrlPntSet TgtCtrlPntsSet(new SimpleCtrlPntSet);
	std::vector<std::pair<int, int> > SrcSurIdxSet;
	std::vector<std::pair<int, int> > TgtSurfIdxSet;
	std::vector<std::pair<double, double> > SrcUVSet;
	std::vector<std::pair<double, double> > TgtUVSet;

	for (unsigned int i=0; i!=SrcCorPointSet.size(); ++i)
	{
		const ControlPoints& CurTgtCtrlPnts = (*TgtCorCtrlPntsSet)[i];

		std::pair<int, int> IntersectedSurfIdx;
		bool IsIntersected = false;
		const unsigned int CtrlMatSpan = 4;
		SRegionIndexRange RegionIndexRange = {0, m_MatRows - CtrlMatSpan, 0, m_MatCols - CtrlMatSpan};
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
	voCorPairSet.first = makeSharedPtr(new CCommonLCSNSSubset<PtrSimpleCtrlPntSet>(SrcCtrlPntsSet));
	voCorPairSet.second = makeSharedPtr(new CCommonLCSNSSubset<PtrSimpleCtrlPntSet>(TgtCtrlPntsSet));
	CCommonLCSNSSubset<PtrSimpleCtrlPntSet>* pPairedCorSrcPntSet = dynamic_cast<CCommonLCSNSSubset<PtrSimpleCtrlPntSet>*>((voCorPairSet.first).get());
	CCommonLCSNSSubset<PtrSimpleCtrlPntSet>* pPairedCorTgtPntSet = dynamic_cast<CCommonLCSNSSubset<PtrSimpleCtrlPntSet>*>((voCorPairSet.second).get());
	pPairedCorSrcPntSet->swapUVSet(SrcSurIdxSet, SrcUVSet);
	pPairedCorTgtPntSet->swapUVSet(TgtSurfIdxSet, TgtUVSet);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CNormalShootingLCSNSClosest::__parseConfig( void )
{
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyLenFact2Move(), m_LenFact2Move);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyClosestDistFact(), m_ClosestDistFactor);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(CControlMatrixLCSNSClosest::getKeySizeCtrlMatrixRow(), m_MatRows);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(CControlMatrixLCSNSClosest::getKeySizeCtrlMatrixCol(), m_MatCols);
	_ASSERT(DumpRes);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CNormalShootingLCSNSClosest::setTgtPointCloud( const hivePointCloud::CPointCloud* vTgtPointCloud )
{
	m_pTgtPntCld = vTgtPointCloud;

	if (!m_pKNN.get())
	{
		m_pKNN.reset(new hiveCommon::CKNNSearch);
	}

	m_pKNN->initKNNSearch(vTgtPointCloud->getPosPointer(), vTgtPointCloud->getNumPoints(), 3, 1);
}
