#include "SimilarityTriangle.h"
#include <cmath>
#include <climits>
#include <algorithm>
#include <boost/typeof/typeof.hpp>
#include <boost/foreach.hpp>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include "HiveCommonMicro.h"
#include "KNNSearch.h"
#include "ProductFactoryData.h"
#include "ControlParameters.h"
#include "ICPMacros.h"
#include "PointCloudSubset.h"
#include "RegMath.h"
#include "RegUtilityFunctions.h"
#include "PointCloud.h"
#include "UniqueData.h"

#ifdef _DEBUG
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "TestUnitity.h"
#endif

using namespace hiveRegistration;

hiveRegistration::CSimilarityTriangle::CSimilarityTriangle(const CIndexSubset* vLoopIdxSet, const CIndexSubset* vMatchIdxSet)
: m_LoopPointCloud(vLoopIdxSet->getPointCloud())
, m_MatchPointCloud(vLoopIdxSet->getPointCloud())
, m_LoopSampleKNNIdxSet(vLoopIdxSet->getIndexSet())
, m_MatchSampleKNNIdxSet(vLoopIdxSet->getIndexSet())
{
	__init();
}

hiveRegistration::CSimilarityTriangle::CSimilarityTriangle(
	const hivePointCloud::CPointCloud& vLoopPointCloud, const hivePointCloud::CPointCloud& vMatchPointCloud,
	const std::vector<unsigned>& vLoopKNN, const std::vector<unsigned>& vMatchKNN)
: m_LoopPointCloud(vLoopPointCloud)
, m_MatchPointCloud(vMatchPointCloud)
, m_LoopSampleKNNIdxSet(vLoopKNN)
, m_MatchSampleKNNIdxSet(vMatchKNN)
{
	__init();
}

CSimilarityTriangle::~CSimilarityTriangle()
{
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CSimilarityTriangle::__init(void)
{
	const double InitNumNeighbor = 10;
	PTR_CONTROL_PARAMS->setIfNotExist(getKeySimSquaredDistFact(), 3.0);
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyNormCosDist(), 0.25);
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyNumNeighbor(), 5);
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyGssCurvTolerance(), 10000.0);

	__parseCtrlParams();
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CSimilarityTriangle::__parseCtrlParams(void)
{
	double NumNeighbor;
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyNumNeighbor(), NumNeighbor);
	_ASSERT(DumpRes);

	double SquareDistFact;
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeySimSquaredDistFact(), SquareDistFact);
	_ASSERT(DumpRes);

	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyNormCosDist(), m_NormCosDist);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyGssCurvTolerance(), m_GssCurvTol);
	_ASSERT(DumpRes);

	double UnitSquareDist = (dynamic_cast<CUniqueData*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(CUniqueData::getSigSrcUniqueData())))->getUniqSquareDist();
	m_SquareDistThreshold = SquareDistFact * UnitSquareDist;
	m_pLoopSampleKNN = __constructSampleKNN(m_LoopPointCloud, m_LoopSampleKNNIdxSet, NumNeighbor);
	m_pMatchSampleKNN = __constructSampleKNN(m_MatchPointCloud, m_MatchSampleKNNIdxSet, NumNeighbor);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CSimilarityTriangle::fillValidCandidate(
	unsigned int vLoopIdx, const std::vector<unsigned int>& vCorCandidateSet, 
	std::vector<unsigned>& voValidCandSet) const
{
	voValidCandSet.clear();

#ifdef _DEBUG
	std::vector<unsigned> LoopNeibs;
	std::vector<unsigned> CandNeibs;
	m_pLoopSampleKNN->executeKNN(m_LoopPointCloud.getPosPointer() + vLoopIdx*3, LoopNeibs);
	__convSubIdxSet2WholeIdxSet(m_LoopSampleKNNIdxSet, LoopNeibs);
	
	static char OutFileName[256];
	sprintf(OutFileName, "TestData\\output\\CorPointPairs\\CoarseReg_Mid\\Neighbors\\%d_loop.ply", vLoopIdx);
	export2Ply(OutFileName, m_LoopPointCloud, LoopNeibs);
#endif

	std::vector<std::pair<double, unsigned> > LoopNeibDistSet;
	std::vector<std::pair<double, unsigned> > MatchNeibDistSet;
	__computeNeibSquareDistSet(vLoopIdx, m_LoopPointCloud, m_LoopSampleKNNIdxSet, m_pLoopSampleKNN.get(), LoopNeibDistSet);

	BOOST_FOREACH (unsigned CandIdx, vCorCandidateSet)
	{
#ifdef _DEBUG
		m_pMatchSampleKNN->executeKNN(m_MatchPointCloud.getPosPointer() + CandIdx*3, CandNeibs);
		__convSubIdxSet2WholeIdxSet(m_MatchSampleKNNIdxSet, CandNeibs);
		sprintf(OutFileName, "TestData\\output\\CorPointPairs\\CoarseReg_Mid\\Neighbors\\%d_%d_match.ply", vLoopIdx, CandIdx);
		export2Ply(OutFileName, m_MatchPointCloud, CandNeibs);
#endif

		MatchNeibDistSet.clear();
		__computeNeibSquareDistSet(CandIdx, m_MatchPointCloud, m_MatchSampleKNNIdxSet, m_pMatchSampleKNN.get(), MatchNeibDistSet);
		__findValidPntThroughCongruentTriangle(LoopNeibDistSet, MatchNeibDistSet, voValidCandSet);
	}
}

//*********************************************************************************
//FUNCTION:
// the same index can be put in "vioTriIdxPairs" multiple times, giving it higher priority when apply RANSAC
void hiveRegistration::CSimilarityTriangle::__findValidPntThroughCongruentTriangle(
	const std::vector<std::pair<double, unsigned> >& vLoopNeibDist, 
	const std::vector<std::pair<double, unsigned> >& vMatchNeibDist, 
	std::vector<unsigned>& voValidCandSet) const
{
	std::vector<std::pair<unsigned, unsigned> > SimVertPairSet;
	for (std::vector<std::pair<double, unsigned> >::const_iterator LoopItr=vLoopNeibDist.begin()+1; LoopItr!=vLoopNeibDist.end(); ++LoopItr)
	{
		for (std::vector<std::pair<double, unsigned> >::const_iterator MatchItr=vMatchNeibDist.begin()+1; MatchItr!=vMatchNeibDist.end(); ++MatchItr)
		{
			if (fabs(LoopItr->first - MatchItr->first) < m_SquareDistThreshold)
			{
				SimVertPairSet.push_back(std::make_pair(LoopItr->second, MatchItr->second));
			}
		}
	}
	for (std::vector<std::pair<unsigned, unsigned> >::const_iterator SimItr=SimVertPairSet.begin(); SimItr!=SimVertPairSet.end(); ++SimItr)
	{
		for (std::vector<std::pair<unsigned, unsigned> >::const_iterator MoveItr=SimItr+1; MoveItr!=SimVertPairSet.end(); ++MoveItr)
		{
			double Loop3rdEdgeLen = comSquaredDist(m_LoopPointCloud.getPosPointer(), SimItr->first, m_LoopPointCloud.getPosPointer(), MoveItr->first);
			double Match3rdEdgeLen = comSquaredDist(m_MatchPointCloud.getPosPointer(), SimItr->second, m_MatchPointCloud.getPosPointer(), MoveItr->second);
			if (fabs(Loop3rdEdgeLen - Match3rdEdgeLen) < m_SquareDistThreshold)
			{
				voValidCandSet.push_back(vMatchNeibDist.front().second);
				return;
				//unsigned LoopTri[3] = { vLoopNeibDist.front().second, SimItr->first, MoveItr->first };
				//unsigned MatchTri[3] = { vMatchNeibDist.front().second, SimItr->second, MoveItr->second };

				//bool IsWithinNormRest0 = __isWithinNormTolerance(LoopTri[0], LoopTri[1], MatchTri[0], MatchTri[1]);
				//bool IsWithinNormRest1 = __isWithinNormTolerance(LoopTri[0], LoopTri[2], MatchTri[0], MatchTri[2]);
				////bool IsWithinGssRest0 = __isWithinGssTolerance(LoopTri[0], MatchTri[0]);
				////bool IsWithinGssRest1 = __isWithinGssTolerance(LoopTri[1], MatchTri[1]);
				////bool IsWithinGssRest2 = __isWithinGssTolerance(LoopTri[2], MatchTri[2]);

				//if (IsWithinNormRest0 && IsWithinNormRest1 /*&& IsWithinGssRest0 && IsWithinGssRest1 && IsWithinGssRest2*/)
				//{
				//	voValidCandSet.push_back(vMatchNeibDist.front().second);
				//	return;
				//}
			}
		}
	}
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CSimilarityTriangle::__computeNeibSquareDistSet(
	unsigned vIdx, 
	const hivePointCloud::CPointCloud& vPointCloud,
	const std::vector<unsigned>& vIdxSet, 
	hiveCommon::CKNNSearch* vKNN, 
	std::vector<std::pair<double, unsigned> >& voNeibDist) const
{
	std::vector<unsigned> NeibSet;
	vKNN->executeKNN(vPointCloud.getPosPointer() + vIdx*3, NeibSet);
	__convSubIdxSet2WholeIdxSet(vIdxSet, NeibSet);

	voNeibDist.clear();
	voNeibDist.push_back(std::make_pair(0, NeibSet.front()));			// FIXME: The first neighbor is itself
	for (std::vector<unsigned>::iterator Itr=NeibSet.begin()+1; Itr!=NeibSet.end(); ++Itr)
	{
		double SquareDist = comSquaredDist(vPointCloud.getPosPointer(), vIdx, vPointCloud.getPosPointer(), *Itr, 3);
		voNeibDist.push_back(std::make_pair(SquareDist, *Itr));
	}
}

//*********************************************************************************
//FUNCTION:
boost::shared_ptr<hiveCommon::CKNNSearch> 
			hiveRegistration::CSimilarityTriangle::__constructSampleKNN(const hivePointCloud::CPointCloud& vPointCloud, const std::vector<unsigned>& vIdxSet, int vNumNeighbor)
{
	const double* pPos = vPointCloud.getPosPointer();
	Eigen::Matrix<double, 3, Eigen::Dynamic> SamplePoints;
	SamplePoints.resize(3, vIdxSet.size());

	for (unsigned int i=0; i<vIdxSet.size(); ++i)
	{
		SamplePoints(0, i) = *(pPos + vIdxSet[i]*3 + 0);
		SamplePoints(1, i) = *(pPos + vIdxSet[i]*3 + 1);
		SamplePoints(2, i) = *(pPos + vIdxSet[i]*3 + 2);
	}

	unsigned int Dimension = 3;
	boost::shared_ptr<hiveCommon::CKNNSearch> Ret = boost::shared_ptr<hiveCommon::CKNNSearch>(new hiveCommon::CKNNSearch);
	Ret->initKNNSearch(SamplePoints.data(), vIdxSet.size(), Dimension, vNumNeighbor);
	return Ret;
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CSimilarityTriangle::__convSubIdxSet2WholeIdxSet(const std::vector<unsigned>& vWholeSampleIdxSet, std::vector<unsigned>& vioIdxSet) const
{
	std::vector<unsigned> TmpVec;
	TmpVec.swap(vioIdxSet);
	BOOST_FOREACH (unsigned i, TmpVec)
	{
		vioIdxSet.push_back(vWholeSampleIdxSet[i]);
	}
}

//*********************************************************************************
//FUNCTION:
bool hiveRegistration::CSimilarityTriangle::__isWithinNormTolerance(unsigned vLoop0, unsigned vLoop1, unsigned vMatch0, unsigned vMatch1) const
{
	const double* pLoopNorm = m_LoopPointCloud.getNormalPointer();
	const double* pMatchNorm = m_MatchPointCloud.getNormalPointer();
	Eigen::Vector3d LoopNorm0(pLoopNorm[vLoop0*3], pLoopNorm[vLoop0*3+1], pLoopNorm[vLoop0*3+2]);
	Eigen::Vector3d LoopNorm1(pLoopNorm[vLoop1*3], pLoopNorm[vLoop1*3+1], pLoopNorm[vLoop1*3+2]);
	Eigen::Vector3d MatchNorm0(pMatchNorm[vMatch0*3], pMatchNorm[vMatch0*3+1], pMatchNorm[vMatch0*3+2]);
	Eigen::Vector3d MatchNorm1(pMatchNorm[vMatch1*3], pMatchNorm[vMatch1*3+1], pMatchNorm[vMatch1*3+2]);
	LoopNorm0.normalize();
	LoopNorm1.normalize();
	MatchNorm0.normalize();
	MatchNorm1.normalize();

	double LoopCosDist = dot(LoopNorm0.data(), 0, LoopNorm1.data(), 0);
	double MatchCosDist = dot(MatchNorm0.data(), 0, MatchNorm1.data(), 0);

	double NormSim = fabs(fabs(LoopCosDist) - fabs(MatchCosDist)) / fabs(fabs(LoopCosDist) + fabs(MatchCosDist));
	return NormSim < m_NormCosDist;
}

//*********************************************************************************
//FUNCTION:
bool hiveRegistration::CSimilarityTriangle::__isWithinGssTolerance(unsigned vLoopIdx, unsigned vMatchIdx) const
{
	const double* pLoopGss = m_LoopPointCloud.getGssCurvaturePointer();
	const double* pMatchGss = m_MatchPointCloud.getGssCurvaturePointer();

	return ( (pLoopGss[vLoopIdx] > 0 && pMatchGss[vMatchIdx] >0) || 
		     (pLoopGss[vLoopIdx] <=0 && pMatchGss[vMatchIdx] <=0) );
	//return fabs(pLoopGss[vLoopIdx] - pMatchGss[vMatchIdx]) < m_GssCurvTol;
}