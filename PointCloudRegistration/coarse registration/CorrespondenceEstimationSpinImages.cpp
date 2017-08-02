#include "CorrespondenceEstimationSpinImages.h"
#ifdef _DEBUG
#include <stdio.h>
#endif
#include <map>
#include <boost/foreach.hpp>
#include <Eigen/LU>
#include "ProductFactory.h"
#include "ProductFactoryData.h"
#include "ControlParameters.h"
#include "ICPMacros.h"
#include "ICPType.h"
#include "RegUtilityFunctions.h"
#include "PointCloudSubset.h"
#include "RegUtilityFunctions.h"
#include "KNNSearch.h"
#include "NewKNNSearch.h"
#include "SamplerExtremeCurvature.h"
#include "SpinImagesGenerator.h"
#include "SimilarityTriangle.h"
#include "LCPTriangle.h"
#include "TransformationEstimationLCP.h"
#include "UniqueData.h"
#include "DecreaseDimensionPCA.h"

#ifdef _DEBUG
#include <fstream>
#include "TestUnitity.h"
#include "bitmap_image.hpp"
#endif

#ifndef _DEBUG
#include <fstream>
#include <Windows.h>
#endif

using namespace hiveRegistration;

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
}

hiveCommon::CProductFactory<CCorrespondenceEstimationSpinImages> TheCreator(CCorrespondenceEstimationSpinImages::getClassSig());
const static std::string g_sSrcSpinImagesSamplerSigASI = boost::to_upper_copy(std::string("SamplerExtremeCurvature_SrcCloudSampler"));
hiveCommon::CProductFactory<CSamplerExtremeCurvature> TheCreatorSrcSpinImagesSampler(g_sSrcSpinImagesSamplerSigASI);


CCorrespondenceEstimationSpinImages::CCorrespondenceEstimationSpinImages()
{
	_letFactoryReleaseProduct();
}

//*********************************************************************************
//FUNCTION:
void CCorrespondenceEstimationSpinImages::_setPointCloudV(const hivePointCloud::CPointCloud* vSourcePointCloud, const hivePointCloud::CPointCloud* vTargetPointCloud)
{
	m_pSourcePointCloud = vSourcePointCloud;
	m_pTargetPointCloud = vTargetPointCloud;
}

//*********************************************************************************
//FUNCTION:
void CCorrespondenceEstimationSpinImages::_determineCorrespondencesV(
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
CIndexSubset* hiveRegistration::CCorrespondenceEstimationSpinImages::__sampleSourcePointCloud(void)
{
	static CorrespondencePointSet CorSrcPointSet;
	if (!CorSrcPointSet.get())
	{
		CBaseSampler* pSampler = dynamic_cast<CBaseSampler*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(g_sSrcSpinImagesSamplerSigASI));
		_ASSERT(pSampler);
		CorSrcPointSet = pSampler->samplePointCloud(*m_pSourcePointCloud);
	}
	CIndexSubset* pSrcCorIndexSet = dynamic_cast<CIndexSubset*>(CorSrcPointSet.get());
	_ASSERT(pSrcCorIndexSet);
	return pSrcCorIndexSet;
}

//*********************************************************************************
//FUNCTION:
void CCorrespondenceEstimationSpinImages::__doLoopMatch(
	CIndexSubset* vLoopIdxSet, CIndexSubset* vMatchIdxSet, 
	std::vector<unsigned int>& voMatchedLoopIdxSet, std::vector<unsigned int>& voMatchedMatchIdxSet)
{
	_ASSERT(vLoopIdxSet && vMatchIdxSet);

	double RetainRange[2];
	std::vector<unsigned> LoopSampleKNN;
	std::vector<unsigned> MatchSampleKNN;
	__restrictLoopSamplePoints(vLoopIdxSet, LoopSampleKNN, RetainRange);
	//__restrictLoopSamplePoints(vMatchIdxSet, LoopSampleKNN, RetainRange);
	__restrictMatchSamplePoints(vMatchIdxSet, MatchSampleKNN, RetainRange);

#ifdef _DEBUG
	export2Ply("TestData\\output\\CorIndices\\CourseReg\\idx_loop_restrict.ply", vLoopIdxSet->getPointCloud(), vLoopIdxSet->getIndexSet());
	export2Ply("TestData\\output\\CorIndices\\CourseReg\\idx_match_restrict.ply", vMatchIdxSet->getPointCloud(), vMatchIdxSet->getIndexSet());
#endif

	unsigned ImageWidth;
	unsigned ImageHeight;
	double   BinSize;
	bool DumRes = PTR_CONTROL_PARAMS->dumpValue(CSpinImagesGenerator::getKeyImageWidth(), ImageWidth);
	_ASSERT(DumRes);
	DumRes = PTR_CONTROL_PARAMS->dumpValue(CSpinImagesGenerator::getKeyImageHeight(), ImageHeight);
	_ASSERT(DumRes);
	DumRes = PTR_CONTROL_PARAMS->dumpValue(CSpinImagesGenerator::getKeyBinSize(), BinSize);
	_ASSERT(DumRes);
	ImageWidth /= BinSize;
	ImageHeight /= BinSize;

	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> LoopSpinImages;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatchSpinImages;

	CSpinImagesGenerator LoopSpinImagesGenerator(vLoopIdxSet->getPointCloud());
	CSpinImagesGenerator MatchSpinImagesGenerator(vMatchIdxSet->getPointCloud());
	LoopSpinImagesGenerator.genSpinImages(vLoopIdxSet->getIndexSet(), LoopSpinImages);
	MatchSpinImagesGenerator.genSpinImages(vMatchIdxSet->getIndexSet(), MatchSpinImages);

	hiveSearch::CKNNSearch SpinImagesKNN;
	SpinImagesKNN.setInputData(MatchSpinImages.data(), MatchSpinImages.cols(), MatchSpinImages.rows());

	std::vector<unsigned> TmpVec;
	std::vector<std::pair<unsigned, std::vector<unsigned> > > IdxGroupSet;
	CSimilarityTriangle SimTriangle(vLoopIdxSet->getPointCloud(), vMatchIdxSet->getPointCloud(), LoopSampleKNN, MatchSampleKNN);
	CLCPTriangle LCPTriangle(vLoopIdxSet->getPointCloud(), vMatchIdxSet->getPointCloud());
	double UnitSquareDist = (dynamic_cast<CUniqueData*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(CUniqueData::getSigSrcUniqueData())))->getUniqSquareDist();
	CTransformationEstimationLCP* pTmpRT = dynamic_cast<CTransformationEstimationLCP*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(CTransformationEstimationLCP::getClassSig()));

#ifndef _DEBUG
	std::ofstream TimeFile("TestData\\output\\SpinImgMatchTime.txt");
	DWORD TimeBegin = GetTickCount();
#endif

	int SizeLoopIdxSet = vLoopIdxSet->getIndexSet().size();
	for (int i=SizeLoopIdxSet - 1; i>=0; --i)
	{
		const std::vector<unsigned>* pNeibs = SpinImagesKNN.executeSearch(m_NumSpinNeib, LoopSpinImages.col(i).data(), 1);
		unsigned LoopIdx = vLoopIdxSet->getIndexSet()[i];
		TmpVec.clear();

#ifdef _DEBUG
		static char OutFileName[256];
		sprintf(OutFileName, "TestData\\output\\CorPointPairs\\CoarseReg_Mid\\all\\%d_loop.ply", LoopIdx);
		export2Ply(OutFileName, vLoopIdxSet->getPointCloud(), LoopIdx);
#endif

		for (std::vector<unsigned>::const_iterator citr=pNeibs->begin(); citr!=pNeibs->end(); ++citr)
		{
			double SquaredDist = comSquaredDist(LoopSpinImages.data(), i, MatchSpinImages.data(), *citr, LoopSpinImages.rows());

			if (SquaredDist < m_SpinDistThreshold)
			{
				unsigned MatchIdx = vMatchIdxSet->getIndexSet()[*citr];
				TmpVec.push_back(MatchIdx);
#ifdef _DEBUG
				sprintf(OutFileName, "TestData\\output\\CorPointPairs\\CoarseReg_Mid\\all\\%d_%d_match.ply", LoopIdx, MatchIdx);
				export2Ply(OutFileName, vMatchIdxSet->getPointCloud(), MatchIdx);
#endif
			}
			else
			{
				break;
			}
		}

		if (!TmpVec.empty())
		{
			std::vector<unsigned> ValidVec;
			if (m_UseSimTri) SimTriangle.fillValidCandidate(LoopIdx, TmpVec, ValidVec);
			else ValidVec.swap(TmpVec);

#ifdef _DEBUG
			if (!ValidVec.empty())
			{
				char FileName[255];
				sprintf(FileName, "TestData\\output\\CorPointPairs\\CoarseReg_Mid\\all\\%d_Match_SimTri.ply", LoopIdx);
				export2Ply(FileName, vMatchIdxSet->getPointCloud(), ValidVec);
			}
#endif

			if (!ValidVec.empty())
			{
				IdxGroupSet.push_back(std::make_pair(LoopIdx, ValidVec));

				Eigen::Matrix3d TmpRot;
				Eigen::Vector3d TmpTra;
 				if (LCPTriangle.comRTWithLastElement(IdxGroupSet, UnitSquareDist, TmpRot, TmpTra))
 				{
 					if ((&(vLoopIdxSet->getPointCloud())) != m_pSourcePointCloud)
 					{
 						pTmpRT->fetchTra() = -TmpRot.inverse() * TmpTra;
 						pTmpRT->fetchRot() = TmpRot.inverse();
 					}
 					else
 					{
 						pTmpRT->fetchTra() = TmpTra;
 						pTmpRT->fetchRot() = TmpRot;
 					}
 
 #ifndef _DEBUG
 					TimeFile << GetTickCount() - TimeBegin << std::endl;
 #endif
 					return;
 				}
			}
		}
	}
#ifndef _DEBUG
	TimeFile << GetTickCount() - TimeBegin << std::endl;
#endif
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CCorrespondenceEstimationSpinImages::__restrictLoopSamplePoints(
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
void hiveRegistration::CCorrespondenceEstimationSpinImages::__restrictMatchSamplePoints(
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
void hiveRegistration::CCorrespondenceEstimationSpinImages::__parseCtrlParams(void)
{
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyMatchGaussianTolerance(), m_GssTol);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyValidSampleRangeMin(), m_SampleRatioRange[0]);
	_ASSERT(DumpRes && m_SampleRatioRange[0] > 0 && m_SampleRatioRange[0] < 1);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyValidSampleRangeMax(), m_SampleRatioRange[1]);
	_ASSERT(DumpRes && m_SampleRatioRange[1] > m_SampleRatioRange[0] && m_SampleRatioRange[1] < 1);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeySampleKNNRangeMin(), m_SampleKNNRange[0]);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeySampleKNNRangeMax(), m_SampleKNNRange[1]);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeySpinDistThreshold(), m_SpinDistThreshold);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyPCADimension(), m_PCADimension);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyNumSpinNeibs(), m_NumSpinNeib);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyUseSimTriangle(), m_UseSimTri);
	_ASSERT(DumpRes);
}