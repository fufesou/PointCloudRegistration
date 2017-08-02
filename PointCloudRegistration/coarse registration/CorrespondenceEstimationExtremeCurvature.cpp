#include "CorrespondenceEstimationExtremeCurvature.h"
#include <algorithm>
#include <map>
#include <boost/foreach.hpp>
#include <Eigen/LU>
#include "ProductFactory.h"
#include "ProductFactoryData.h"
#include "ControlParameters.h"
#include "ICPMacros.h"
#include "ICPType.h"
#include "RegUtilityFunctions.h"
#include "SimilarityTriangle.h"
#include "PointCloudSubset.h"
#include "RegUtilityFunctions.h"
#include "KNNSearch.h"
#include "SamplerExtremeCurvature.h"
#include "LCPTriangle.h"
#include "TransformationEstimationLCP.h"
#include "UniqueData.h"

#ifdef _DEBUG
#include <fstream>
#include "TestUnitity.h"
#endif

#define USE_ONLY_POINT_MATCH

namespace
{
	template<typename T>
	struct IdxCmp 
	{
		const T* Data;
		IdxCmp(const T* vData) : Data(vData) {}

		template<typename TIdx>
		bool operator()(TIdx vL, TIdx vR)
		{
			return fabs(Data[vL]) < fabs(Data[vR]);
		}
		template<typename TIdx>
		bool operator()(TIdx vL, double vR)
		{
			return fabs(Data[vL]) < fabs(vR);
		}
		template<typename TIdx>
		bool operator()(double vL, TIdx vR)
		{
			return fabs(vL) < fabs(Data[vR]);
		}
	};

	bool cmpPairIdx(const std::pair<unsigned, unsigned>& vL, const std::pair<unsigned, unsigned>& vR)
	{
		return vL.first < vR.first;
	}
}

using namespace hiveRegistration;

hiveCommon::CProductFactory<CCorrespondenceEstimationExtremeCurvature> TheCreator(CCorrespondenceEstimationExtremeCurvature::getClassSig());
const static std::string g_SrcExtremeCurvSamplerSig = boost::to_upper_copy(std::string("SamplerExtremeCurvature_SrcCloudSampler"));
hiveCommon::CProductFactory<CSamplerExtremeCurvature> TheCreatorSrcExtremeCurvSampler(g_SrcExtremeCurvSamplerSig);


CCorrespondenceEstimationExtremeCurvature::CCorrespondenceEstimationExtremeCurvature()
{
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyFistTolerance(), 0.0001);
	PTR_CONTROL_PARAMS->setIfNotExist(getKeySecondTolerance(), 0.0001);
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyMatchGaussianTolerance(), 1200);
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyCentroidNumPnts(), 40);
	PTR_CONTROL_PARAMS->setIfNotExist(getKeySqureDist2CentFact(), 2.0);
	PTR_CONTROL_PARAMS->setIfNotExist(getKeySampleKNNRangeMin(), 0.40);
	PTR_CONTROL_PARAMS->setIfNotExist(getKeySampleKNNRangeMax(), 0.99);
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyValidSampleRangeMin(), 0.50);
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyValidSampleRangeMax(), 0.95);

	_letFactoryReleaseProduct();
}

//*********************************************************************************
//FUNCTION:
void CCorrespondenceEstimationExtremeCurvature::_setPointCloudV(const hivePointCloud::CPointCloud* vSourcePointCloud, const hivePointCloud::CPointCloud* vTargetPointCloud)
{
	m_pSourcePointCloud = vSourcePointCloud;
	m_pTargetPointCloud = vTargetPointCloud;

	m_UnitSquareDist = (dynamic_cast<CUniqueData*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(CUniqueData::getSigSrcUniqueData())))->getUniqSquareDist();
}

//*********************************************************************************
//FUNCTION:
void CCorrespondenceEstimationExtremeCurvature::_determineCorrespondencesV(
	const CorrespondencePointSet& vTgtCorPointSet,
	const Eigen::Matrix3d& vR,
	const ColVector3& vT,
	CorrespondencePairSet& voCorrespondencePairs)
{
	CIndexSubset* pTgtCorIndexSet = dynamic_cast<CIndexSubset*>(vTgtCorPointSet.get());
	_ASSERT(pTgtCorIndexSet);
	CIndexSubset* pSrcCorIndexSet = __sampleSourcePointCloud();

#ifdef _DEBUG
	export2Ply("TestData\\output\\CorIndices\\CourseReg\\idx_src.ply", *m_pSourcePointCloud, pSrcCorIndexSet->getIndexSet());
	export2Ply("TestData\\output\\CorIndices\\CourseReg\\idx_tgt.ply", *m_pTargetPointCloud, pTgtCorIndexSet->getIndexSet());
#endif

	__parseCtrlParams();

	std::vector<unsigned int> MatchedLoopIdxSet;
	std::vector<unsigned int> MatchedMatchIdxSet;
	if (pSrcCorIndexSet->getIndexSet().size() > pTgtCorIndexSet->getIndexSet().size())
	{
		__doLoopMatch(pTgtCorIndexSet, pSrcCorIndexSet, MatchedLoopIdxSet, MatchedMatchIdxSet);
		voCorrespondencePairs.first = makeSharedPtr(new CIndexSubset(*m_pSourcePointCloud, MatchedMatchIdxSet));
		voCorrespondencePairs.second = makeSharedPtr(new CIndexSubset(*m_pTargetPointCloud, MatchedLoopIdxSet));
	}
	else
	{
		__doLoopMatch(pSrcCorIndexSet, pTgtCorIndexSet, MatchedLoopIdxSet, MatchedMatchIdxSet);
		voCorrespondencePairs.first = makeSharedPtr(new CIndexSubset(*m_pSourcePointCloud, MatchedLoopIdxSet));
		voCorrespondencePairs.second = makeSharedPtr(new CIndexSubset(*m_pTargetPointCloud, MatchedMatchIdxSet));
	}
}

//*********************************************************************************
//FUNCTION:
CIndexSubset* hiveRegistration::CCorrespondenceEstimationExtremeCurvature::__sampleSourcePointCloud(void)
{
	static CorrespondencePointSet CorSrcPointSet;
	if (!CorSrcPointSet.get())
	{
		CSamplerExtremeCurvature* pSampler = dynamic_cast<CSamplerExtremeCurvature*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(g_SrcExtremeCurvSamplerSig));
		_ASSERT(pSampler);
		CorSrcPointSet = pSampler->samplePointCloud(*m_pSourcePointCloud);
	}
	CIndexSubset* pSrcCorIndexSet = dynamic_cast<CIndexSubset*>(CorSrcPointSet.get());
	_ASSERT(pSrcCorIndexSet);
	return pSrcCorIndexSet;
}

//*********************************************************************************
//FUNCTION:
hiveRegistration::ColVector3 CCorrespondenceEstimationExtremeCurvature::__computeNeiborsCentroid(unsigned vIdx, boost::shared_ptr<hiveCommon::CKNNSearch> vKNN, const hivePointCloud::CPointCloud& vPointCloud)
{
	std::vector<unsigned> NeibIdxSet;
	vKNN->executeKNN(vPointCloud.getPosPointer() + vIdx*3, NeibIdxSet);

	const double* pPos = vPointCloud.getPosPointer();

	double Sum[3] = { 0, 0 };
	BOOST_FOREACH (unsigned i, NeibIdxSet)
	{
		Sum[0] += pPos[i*3];
		Sum[1] += pPos[i*3 + 1];
		Sum[2] += pPos[i*3 + 2];
	}
	return ColVector3( Sum[0] / NeibIdxSet.size(), Sum[1] / NeibIdxSet.size(), Sum[2] / NeibIdxSet.size() );
}

//*********************************************************************************
//FUNCTION:
void CCorrespondenceEstimationExtremeCurvature::__setSimMeasurement(const std::vector<unsigned>& vLoopKNN, const std::vector<unsigned>& vMatchKNN)
{
	m_pSimTriangle = boost::shared_ptr<CSimilarityTriangle>(new CSimilarityTriangle(*m_pLoopPointCloud, *m_pMatchPointCloud, vLoopKNN, vMatchKNN));
}

//*********************************************************************************
//FUNCTION:
void CCorrespondenceEstimationExtremeCurvature::__constructPointCloudKNN(const hivePointCloud::CPointCloud& vPointCloud, boost::shared_ptr<hiveCommon::CKNNSearch>& voKNN)
{
	voKNN.reset(new hiveCommon::CKNNSearch);
	voKNN->initKNNSearch(vPointCloud.getPosPointer(), vPointCloud.getNumPoints(), 3, m_CentNumNeibs);
}

//*********************************************************************************
//FUNCTION:
void CCorrespondenceEstimationExtremeCurvature::__setLoopMatchStructData(const std::vector<unsigned>& vLoopKNN, const std::vector<unsigned>& vMatchKNN)
{
	__setSimMeasurement(vLoopKNN, vMatchKNN);
	__constructPointCloudKNN(*m_pLoopPointCloud, m_pLoopKNN);
	__constructPointCloudKNN(*m_pMatchPointCloud, m_pMatchKNN);
}

//*********************************************************************************
//FUNCTION:
void CCorrespondenceEstimationExtremeCurvature::__restrictCentroid(unsigned vLoopIdx, std::vector<unsigned>& vioCorCandSet)
{
	//static std::map<unsigned, ColVector3> MapCandIdxCentroid;		//FIXME: 加速结构，可能没用，暂时不考虑
	double SquareDist2CentThreshold = m_Dist2CentFact * m_UnitSquareDist;

	const ColVector3 CurLoopCent = __computeNeiborsCentroid(vLoopIdx, m_pLoopKNN, *m_pLoopPointCloud);
	double LoopSquareDist = comSquaredDist(m_pLoopPointCloud->getPosPointer(), vLoopIdx, CurLoopCent.data(), 0, 3);

	std::vector<unsigned> InitCand;
	InitCand.swap(vioCorCandSet);
	BOOST_FOREACH (unsigned i, InitCand)
	{
		const ColVector3 CurMatchCent = __computeNeiborsCentroid(i, m_pMatchKNN, *m_pMatchPointCloud);
		double MatchSquareDist = comSquaredDist(m_pMatchPointCloud->getPosPointer(), i, CurMatchCent.data(), 0, 3);

		_HIVE_SIMPLE_IF(fabs(LoopSquareDist - MatchSquareDist) < SquareDist2CentThreshold, vioCorCandSet.push_back(i));
	}
}

//*********************************************************************************
//FUNCTION:
void CCorrespondenceEstimationExtremeCurvature::__doLoopMatch(
	CIndexSubset* vLoopIdxSet, CIndexSubset* vMatchIdxSet, 
	std::vector<unsigned int>& voMatchedLoopIdxSet, std::vector<unsigned int>& voMatchedMatchIdxSet)
{
	_ASSERT(vLoopIdxSet && vMatchIdxSet);

	m_pLoopPointCloud = &(vLoopIdxSet->getPointCloud());
	m_pMatchPointCloud = &(vMatchIdxSet->getPointCloud());

	double RetainRange[2];
	std::vector<unsigned> LoopSampleKNN;
	std::vector<unsigned> MatchSampleKNN;
	__restrictLoopSamplePoints(vLoopIdxSet, LoopSampleKNN, RetainRange);
	__restrictMatchSamplePoints(vMatchIdxSet, MatchSampleKNN, RetainRange);

#ifdef _DEBUG
	export2Ply("TestData\\output\\CorIndices\\CourseReg\\idx_loop_restrict.ply", *m_pLoopPointCloud, vLoopIdxSet->getIndexSet());
	export2Ply("TestData\\output\\CorIndices\\CourseReg\\idx_match_restrict.ply", *m_pMatchPointCloud, vMatchIdxSet->getIndexSet());
#endif

	__setLoopMatchStructData(LoopSampleKNN, MatchSampleKNN);

	const std::vector<unsigned int>& LoopIdxSet = vLoopIdxSet->getIndexSet();
	std::vector<unsigned int> InitCandidateSet;
	std::vector<unsigned> RestrictedCandSet;
	std::vector<std::pair<unsigned, std::vector<unsigned> > > IdxGroupSet;

	CLCPTriangle LCPTriangle(*m_pLoopPointCloud, *m_pMatchPointCloud);
	CTransformationEstimationLCP* pTmpRT = dynamic_cast<CTransformationEstimationLCP*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(CTransformationEstimationLCP::getClassSig()));
	_ASSERT(pTmpRT);

	unsigned int MatchCount = 0;
	int SizeLoopIdxSet = LoopIdxSet.size();
	for (int i=SizeLoopIdxSet-1; i>=0; --i)
	{
		unsigned int LoopIdx = LoopIdxSet[i];

		if (__matchPoint(LoopIdx, vMatchIdxSet->getIndexSet(), InitCandidateSet))
		{
#ifdef _DEBUG
			static char OutFileName[256];
			sprintf(OutFileName, "TestData\\output\\CorPointPairs\\CoarseReg_Mid\\all\\%d_loop.ply", LoopIdx);
			export2Ply(OutFileName, *m_pLoopPointCloud, LoopIdx);

			sprintf(OutFileName, "TestData\\output\\CorPointPairs\\CoarseReg_Mid\\all\\%d_match.ply", LoopIdx);
			export2Ply(OutFileName, *m_pMatchPointCloud, InitCandidateSet);
#endif

			if (m_UseDist2CentRestrection)
			{
 				__restrictCentroid(LoopIdx, InitCandidateSet);
	 
 #ifdef _DEBUG
			sprintf(OutFileName, "TestData\\output\\CorPointPairs\\CoarseReg_Mid\\all\\%d_match_cent.ply", LoopIdx);
			export2Ply(OutFileName, *m_pMatchPointCloud, InitCandidateSet);
 #endif
			}

			++MatchCount;

			if (m_UseSimTri)
			{
				m_pSimTriangle->fillValidCandidate(LoopIdx, InitCandidateSet, RestrictedCandSet);
			}
			else
			{
				RestrictedCandSet.swap(InitCandidateSet);
			}


			if (!RestrictedCandSet.empty())
			{
#ifdef _DEBUG
				sprintf(OutFileName, "TestData\\output\\CorPointPairs\\CoarseReg_Mid\\all\\%d_match_SimTri.ply", LoopIdx);
				export2Ply(OutFileName, *m_pMatchPointCloud, InitCandidateSet);
#endif

				IdxGroupSet.push_back(std::make_pair(LoopIdx, RestrictedCandSet));

				Eigen::Matrix3d TmpRot;
				Eigen::Vector3d TmpTra;
				if (LCPTriangle.comRTWithLastElement(IdxGroupSet, m_UnitSquareDist, TmpRot, TmpTra))
				{
					if (m_pLoopPointCloud != m_pSourcePointCloud)
					{
						pTmpRT->fetchTra() = -TmpRot.inverse() * TmpTra;
						pTmpRT->fetchRot() = TmpRot.inverse();
					}
					else
					{
						pTmpRT->fetchTra() = TmpTra;
						pTmpRT->fetchRot() = TmpRot;
					}

#ifdef _DEBUG
					for (unsigned i=0; i<IdxGroupSet.size(); ++i)
					{
						char OutFileName[256];
						unsigned LoopIdx = IdxGroupSet[i].first;
						sprintf(OutFileName, "TestData\\output\\CorPointPairs\\CoarseReg_Mid\\all\\%d_0_pnts.ply", LoopIdx);
						export2Ply(OutFileName, vMatchIdxSet->getPointCloud(), IdxGroupSet[i].second);
					}
#endif

					return;
				}
			}
		}
	}
}

//*********************************************************************************
//FUNCTION:
bool CCorrespondenceEstimationExtremeCurvature::__matchPoint(unsigned int vLoopIdx, const std::vector<unsigned int>& vMatchIdxSet, std::vector<unsigned int>& voCandidateSet)
{
	voCandidateSet.clear();

	const double* pLoopK1  = m_pLoopPointCloud->getK1CurvaturePointer();
	const double* pLoopK2  = m_pLoopPointCloud->getK2CurvaturePointer();
	const double* pLoopGss = m_pLoopPointCloud->getGssCurvaturePointer();
	const double* pMatchK1 = m_pMatchPointCloud->getK1CurvaturePointer();
	const double* pMatchK2 = m_pMatchPointCloud->getK2CurvaturePointer();
	const double* pMatchGss = m_pMatchPointCloud->getGssCurvaturePointer();

	const double LoopK1 = *(pLoopK1 + vLoopIdx);
	const double LoopK2 = *(pLoopK2 + vLoopIdx);

	for (unsigned int i=0; i<vMatchIdxSet.size(); ++i)
	{
		unsigned TmpIdx = vMatchIdxSet[i];
		if (__isWithinTolerance(LoopK1, LoopK2, pMatchK1[TmpIdx], pMatchK2[TmpIdx]))
		{
			voCandidateSet.push_back(TmpIdx);
		}
	}

	//const double* pLoopGss = m_pLoopPointCloud->getGssCurvaturePointer();
	//const double* pMatchGss = m_pMatchPointCloud->getGssCurvaturePointer();
	//for (unsigned i=0; i<vMatchIdxSet.size(); ++i)
	//{
	//	unsigned TmpIdx = vMatchIdxSet[i];
	//	if (__isWithinGaussianTolerance(pLoopGss[vLoopIdx], pMatchGss[TmpIdx]))
	//	{
	//		voCandidateSet.push_back(TmpIdx);
	//	}
	//}

	//const double* pLoopGss = m_pLoopPointCloud->getGssCurvaturePointer();
	//const double* pMatchGss = m_pMatchPointCloud->getGssCurvaturePointer();
	//for (unsigned i=0; i<vMatchIdxSet.size(); ++i)
	//{
	//	unsigned TmpIdx = vMatchIdxSet[i];
	//	if ((pLoopGss[vLoopIdx] * pMatchGss[TmpIdx]) > 0)
	//	{
	//		voCandidateSet.push_back(TmpIdx);
	//	}
	//}

	return voCandidateSet.size() != 0;
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CCorrespondenceEstimationExtremeCurvature::__restrictLoopSamplePoints(
	CIndexSubset* vioInitSamplePoints, 
	std::vector<unsigned>& voLoopKNN, 
	double voRetainRange[2]) const
{
	_ASSERT(vioInitSamplePoints);

	const double* pGaussCurv = vioInitSamplePoints->getPointCloud().getGssCurvaturePointer();
	std::vector<unsigned> InitSampleIdxSet = vioInitSamplePoints->getIndexSet();
	std::sort(InitSampleIdxSet.begin(), InitSampleIdxSet.end(), IdxCmp<double>(pGaussCurv));

#ifdef _DEBUG		// FIXME: use for experiment data
	std::ofstream InitCurvOutFile("TestData\\output\\CorIndices\\CourseReg\\init_loop_idx.txt");
	BOOST_FOREACH (unsigned idx, InitSampleIdxSet)
	{
		InitCurvOutFile << pGaussCurv[idx] << std::endl;
	}
	InitCurvOutFile.close();
#endif

	unsigned ValidIdxRange[2] = { InitSampleIdxSet.size() * m_SampleRatioRange[0], InitSampleIdxSet.size() * m_SampleRatioRange[1] };
	unsigned KNNIdxRange[2] = { InitSampleIdxSet.size() * m_SampleKNNRange[0], InitSampleIdxSet.size()* m_SampleKNNRange[1] };
	vioInitSamplePoints->swapIndexSet(std::vector<unsigned>(InitSampleIdxSet.begin() + ValidIdxRange[0], InitSampleIdxSet.begin() + ValidIdxRange[1]));
	voLoopKNN.swap(std::vector<unsigned>(InitSampleIdxSet.begin() + KNNIdxRange[0], InitSampleIdxSet.begin() + KNNIdxRange[1]));
	voRetainRange[0] = fabs(pGaussCurv[InitSampleIdxSet[ValidIdxRange[0]]]) - m_GssTol;		// FIXME: attention, the range is expanded by the length of gaussian tolerance
	voRetainRange[1] = fabs(pGaussCurv[InitSampleIdxSet[ValidIdxRange[1]]]) + m_GssTol;

#ifdef _DEBUG    // FIXME: use for experiment data
	std::ofstream RestrictCurvOutFile("TestData\\output\\CorIndices\\CourseReg\\restrict_loop_idx.txt");
	BOOST_FOREACH (unsigned idx, vioInitSamplePoints->getIndexSet())
	{
		RestrictCurvOutFile << pGaussCurv[idx] << std::endl;
	}
	RestrictCurvOutFile.close();
#endif
}

//*********************************************************************************
//FUNCTION:
void CCorrespondenceEstimationExtremeCurvature::__restrictMatchSamplePoints(
	CIndexSubset* vioInitSamplePoints, 
	std::vector<unsigned>& voMatchKNN, 
	const double vRetainRange[2]) const
{
	_ASSERT(vioInitSamplePoints);

	const double* pGaussCurv = vioInitSamplePoints->getPointCloud().getGssCurvaturePointer();
	const std::vector<unsigned>& InitSampleIdxSet = vioInitSamplePoints->getIndexSet();

	std::vector<unsigned> InitKNNIdxSet = vioInitSamplePoints->getIndexSet();
	std::sort(InitKNNIdxSet.begin(), InitKNNIdxSet.end(), IdxCmp<double>(pGaussCurv));

	unsigned KNNIdxRange[2] = { InitKNNIdxSet.size() * m_SampleKNNRange[0], InitKNNIdxSet.size()* m_SampleKNNRange[1] };
	voMatchKNN.swap(std::vector<unsigned>(InitKNNIdxSet.begin() + KNNIdxRange[0], InitKNNIdxSet.begin() + KNNIdxRange[1]));

#ifdef _DEBUG		// FIXME: use for experiment data
	std::ofstream InitCurvOutFile("TestData\\output\\CorIndices\\CourseReg\\init_match_idx.txt");
	BOOST_FOREACH (unsigned idx, InitKNNIdxSet)
	{
		InitCurvOutFile << pGaussCurv[idx] << std::endl;
	}
	InitCurvOutFile.close();
#endif

	std::vector<unsigned> RetainIdxSet;
#ifdef _DEBUG
	BOOST_FOREACH (unsigned Idx, InitKNNIdxSet)
#else
	BOOST_FOREACH (unsigned Idx, InitSampleIdxSet)
#endif
	{
		if (fabs(pGaussCurv[Idx]) < vRetainRange[1] && fabs(pGaussCurv[Idx]) > vRetainRange[0])
		{
			RetainIdxSet.push_back(Idx);
		}
	}
	vioInitSamplePoints->swapIndexSet(RetainIdxSet);

#ifdef _DEBUG    // FIXME: use for experiment data
	std::ofstream RestrictCurvOutFile("TestData\\output\\CorIndices\\CourseReg\\restrict_match_idx.txt");
	BOOST_FOREACH (unsigned idx, vioInitSamplePoints->getIndexSet())
	{
		RestrictCurvOutFile << pGaussCurv[idx] << std::endl;
	}
	RestrictCurvOutFile.close();
#endif
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CCorrespondenceEstimationExtremeCurvature::__parseCtrlParams(void)
{
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyFistTolerance(), m_FirstTol);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeySecondTolerance(), m_SecondTol);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyMatchGaussianTolerance(), m_GssTol);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyValidSampleRangeMin(), m_SampleRatioRange[0]);
	_ASSERT(DumpRes && m_SampleRatioRange[0] > 0 && m_SampleRatioRange[0] < 1);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyValidSampleRangeMax(), m_SampleRatioRange[1]);
	_ASSERT(DumpRes && m_SampleRatioRange[1] > m_SampleRatioRange[0] && m_SampleRatioRange[1] < 1);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeySampleKNNRangeMin(), m_SampleKNNRange[0]);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeySampleKNNRangeMax(), m_SampleKNNRange[1]);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyUseDist2CentRestrection(), m_UseDist2CentRestrection);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeySqureDist2CentFact(), m_Dist2CentFact);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyCentroidNumPnts(), m_CentNumNeibs);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyUseSimTriangle(), m_UseSimTri);
	_ASSERT(DumpRes);
}