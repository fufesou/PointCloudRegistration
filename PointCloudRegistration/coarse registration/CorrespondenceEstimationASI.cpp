#include "CorrespondenceEstimationASI.h"
#include <boost/typeof/typeof.hpp>
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
#include <iostream>
#include <fstream>
#include "TestUnitity.h"
#include "bitmap_image.hpp"
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

hiveCommon::CProductFactory<CCorrespondenceEstimationASI> TheCreator(CCorrespondenceEstimationASI::getClassSig());
const static std::string g_sSrcSpinImagesSamplerSigASI = boost::to_upper_copy(std::string("SamplerExtremeCurvature_SrcCloudSampler"));
hiveCommon::CProductFactory<CSamplerExtremeCurvature> TheCreatorSrcSpinImagesSamplerASI(g_sSrcSpinImagesSamplerSigASI);


CCorrespondenceEstimationASI::CCorrespondenceEstimationASI()
{
	_letFactoryReleaseProduct();
	__parseCtrlParams();
}

//*********************************************************************************
//FUNCTION:
void CCorrespondenceEstimationASI::_setPointCloudV(const hivePointCloud::CPointCloud* vSourcePointCloud, const hivePointCloud::CPointCloud* vTargetPointCloud)
{
	m_pSourcePointCloud = vSourcePointCloud;
	m_pTargetPointCloud = vTargetPointCloud;
}

//*********************************************************************************
//FUNCTION:
void CCorrespondenceEstimationASI::_determineCorrespondencesV(
	const CorrespondencePointSet& vTgtCorPointSet,
	const Eigen::Matrix3d& vR,
	const ColVector3& vT,
	CorrespondencePairSet& voCorrespondencePairs)
{
	CIndexSubset* pTgtIdxSubset = dynamic_cast<CIndexSubset*>(vTgtCorPointSet.get());
	_ASSERT(pTgtIdxSubset);
	CIndexSubset* pSrcIdxSubset = __sampleSourcePointCloud();

#ifdef _DEBUG
	export2Ply("TestData\\output\\CorIndices\\CourseReg\\idx_src.ply", *m_pSourcePointCloud, pSrcIdxSubset->getIndexSet());
	export2Ply("TestData\\output\\CorIndices\\CourseReg\\idx_tgt.ply", *m_pTargetPointCloud, pTgtIdxSubset->getIndexSet());
#endif

	__resetSamplePointSetByRange(pSrcIdxSubset, pTgtIdxSubset);

	std::vector<unsigned int> MatchedLoopIdxSet;
	std::vector<unsigned int> MatchedMatchIdxSet;
	if (pSrcIdxSubset->getIndexSet().size() > pTgtIdxSubset->getIndexSet().size())
	{
		__doLoopMatch(pTgtIdxSubset, pSrcIdxSubset, MatchedLoopIdxSet, MatchedMatchIdxSet);
		voCorrespondencePairs.first = makeSharedPtr(new CIndexSubset(*m_pSourcePointCloud, MatchedMatchIdxSet));
		voCorrespondencePairs.second = makeSharedPtr(new CIndexSubset(*m_pTargetPointCloud, MatchedLoopIdxSet));
	}
	else
	{
		__doLoopMatch(pSrcIdxSubset, pTgtIdxSubset, MatchedLoopIdxSet, MatchedMatchIdxSet);
		voCorrespondencePairs.first = makeSharedPtr(new CIndexSubset(*m_pSourcePointCloud, MatchedLoopIdxSet));
		voCorrespondencePairs.second = makeSharedPtr(new CIndexSubset(*m_pTargetPointCloud, MatchedMatchIdxSet));
	}
}

//*********************************************************************************
//FUNCTION:
CIndexSubset* hiveRegistration::CCorrespondenceEstimationASI::__sampleSourcePointCloud(void)
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
void CCorrespondenceEstimationASI::__doLoopMatch(
	CIndexSubset* vLoopIdxSet, CIndexSubset* vMatchIdxSet, 
	std::vector<unsigned int>& voMatchedLoopIdxSet, std::vector<unsigned int>& voMatchedMatchIdxSet)
{
	_ASSERT(vLoopIdxSet && vMatchIdxSet);

	CLCPTriangle LCPTriangle(vLoopIdxSet->getPointCloud(), vMatchIdxSet->getPointCloud());
	boost::shared_ptr<CSimilarityTriangle> pSimTri = __createSimilarityTriangle(vLoopIdxSet, vMatchIdxSet);

	std::vector<unsigned int>::const_iterator MatchItrRange[2];
	MatchItrRange[1] = vMatchIdxSet->getIndexSet().end();
	double SectionRatio = __comStepRatio(vMatchIdxSet->getIndexSet().size());
	while (MatchItrRange[1] > vMatchIdxSet->getIndexSet().begin())
	{
#ifdef _DEBUG
		unsigned int TestCurPos = MatchItrRange[1] - vMatchIdxSet->getIndexSet().begin();
#endif

		double CurCurvRange[2];
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatchSpinImagesData;
		__comCurMatchItrRange(vMatchIdxSet, SectionRatio, MatchItrRange);
		__comCurMatchSpinImgs(vMatchIdxSet, MatchItrRange, MatchSpinImagesData);
		__comCurGssCurvRange(vMatchIdxSet, MatchItrRange, CurCurvRange);
		hiveSearch::CKNNSearch SpinImagesKNN;
		SpinImagesKNN.setInputData(MatchSpinImagesData.data(), MatchSpinImagesData.cols(), MatchSpinImagesData.rows());

		unsigned int CurIdxRange[2];
		__setCurLoopIdxRange(CurCurvRange, vLoopIdxSet, CurIdxRange);

		for (unsigned int i=CurIdxRange[1]; i>=CurIdxRange[0]; --i)
		{
			boost::shared_ptr<Eigen::Matrix<double, Eigen::Dynamic, 1> > CurLoopImg = __getOrCreateCurCentImg(i, vLoopIdxSet);
			const std::vector<unsigned>* pNeibs = SpinImagesKNN.executeSearch(m_NumSpinNeibs, CurLoopImg->data(), 1);
			std::vector<unsigned> TmpVec, ValidVec;
			unsigned int CurLoopIdx = vLoopIdxSet->getIndexSet()[i];

			for (std::vector<unsigned>::const_iterator CItr=pNeibs->begin(); CItr!=pNeibs->end(); ++CItr)
			{
				unsigned int MatchIdxOffset = *CItr;
				TmpVec.push_back(*(MatchItrRange[0] + *CItr));
			}

			if (m_UseSimTri)
			{
				pSimTri->fillValidCandidate(CurLoopIdx, TmpVec, ValidVec);
			}
			else
			{
				ValidVec.swap(TmpVec);
			}

			if (!ValidVec.empty())
			{
#ifdef _DEBUG
				char FileName[255];
				sprintf(FileName, "TestData\\output\\CorPointPairs\\CoarseReg_Mid\\all\\%d.ply", CurLoopIdx);
				export2Ply(FileName, vLoopIdxSet->getPointCloud(), CurLoopIdx);
				for (std::vector<unsigned>::const_iterator CItr=ValidVec.begin(); CItr!=ValidVec.end(); ++CItr)
				{
					sprintf(FileName, "TestData\\output\\CorPointPairs\\CoarseReg_Mid\\all\\%d_%d.ply", CurLoopIdx, *CItr);
					export2Ply(FileName, vMatchIdxSet->getPointCloud(), *CItr);
				}
#endif

				m_VecIdxGrpPairSet.push_back(std::make_pair(CurLoopIdx, ValidVec));
				Eigen::Matrix3d TmpRot;
				Eigen::Vector3d TmpTra;
				if (__comCurRT(m_VecIdxGrpPairSet, vLoopIdxSet, LCPTriangle, TmpRot, TmpTra))
				{
					return;
				}
			}
		}

		CurIdxRange[1] = CurIdxRange[0];
	}

#ifdef _DEBUG
	std::cout << "match failed!" << std::endl;
#endif
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CCorrespondenceEstimationASI::__parseCtrlParams(void)
{
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyExtendGaussianRange(), m_ExtGssRange);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeySimilaritySampleRangeMin(), m_SimSmpRange[0]);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeySimilaritySampleRangeMax(), m_SimSmpRange[1]);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeySpinDistThreshold(), m_SpinDistThreshold);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyNumMatchSection(), m_NumMatchSection);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyMaxNumMatchPntEachStep(), m_MaxNumMatchPntEachStep);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyNumSpinNeibs(), m_NumSpinNeibs);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyExtendGaussianRangeFactor(), m_ExtendGssRangeFactor);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyUseSimTriangle(), m_UseSimTri);
	_ASSERT(DumpRes);

	DumpRes = PTR_CONTROL_PARAMS->dumpValue(CSpinImagesGenerator::getKeyImageWidth(), m_ImageWidth);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(CSpinImagesGenerator::getKeyImageHeight(), m_ImageHeight);
	_ASSERT(DumpRes);
	double BinSize;
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(CSpinImagesGenerator::getKeyBinSize(), BinSize);
	_ASSERT(DumpRes);

	m_ImageWidth /= BinSize;
	m_ImageHeight /= BinSize;
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CCorrespondenceEstimationASI::__setSimilaritySamplePointSet(const CIndexSubset* vSamplePoints, std::vector<unsigned>& voSubset) const
{
	_ASSERT(vSamplePoints);

	const double* pGaussCurv = vSamplePoints->getPointCloud().getGssCurvaturePointer();
	const std::vector<unsigned>& InitIdxSet = vSamplePoints->getIndexSet();
	unsigned IdxRange[2] = { InitIdxSet.size() * m_SimSmpRange[0], InitIdxSet.size()* m_SimSmpRange[1] };
	voSubset.swap(std::vector<unsigned>(InitIdxSet.begin() + IdxRange[0], InitIdxSet.begin() + IdxRange[1]));
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CCorrespondenceEstimationASI::__resetSamplePointSetByRange(CIndexSubset* vioSrcSubset, CIndexSubset* vioTgtSubset) const
{
	_ASSERT(vioSrcSubset && vioTgtSubset);

	const double* pGssSrc = vioSrcSubset->getPointCloud().getGssCurvaturePointer();
	const double* pGssTgt = vioTgtSubset->getPointCloud().getGssCurvaturePointer();
	std::vector<unsigned> SrcIdxSubset = vioSrcSubset->getIndexSet();
	std::vector<unsigned> TgtIdxSubset = vioTgtSubset->getIndexSet();

	_ASSERT(pGssSrc && pGssTgt && SrcIdxSubset.size() > 10 && TgtIdxSubset.size() > 10);

#ifdef _DEBUG
	std::ofstream InitSrcGssFile("TestData\\output\\Gss\\SrcCurv_init.txt");
	std::ofstream InitTgtGssFile("TestData\\output\\Gss\\TgtCurv_init.txt");
	for (std::vector<unsigned int>::const_iterator CItr=SrcIdxSubset.begin(); CItr!=SrcIdxSubset.end(); ++CItr)
	{
		InitSrcGssFile << pGssSrc[*CItr] << std::endl;
	}
	for (std::vector<unsigned int>::const_iterator CItr=TgtIdxSubset.begin(); CItr!=TgtIdxSubset.end(); ++CItr)
	{
		InitTgtGssFile << pGssTgt[*CItr] << std::endl;
	}
	InitSrcGssFile.close();
	InitTgtGssFile.close();
#endif

	std::sort(SrcIdxSubset.begin(), SrcIdxSubset.end(), IdxCmp<double>(pGssSrc));
	std::sort(TgtIdxSubset.begin(), TgtIdxSubset.end(), IdxCmp<double>(pGssTgt));

	double SrcMinGss = (fabs(pGssSrc[SrcIdxSubset.front()]) - m_ExtGssRange) > 0 ? (fabs(pGssSrc[SrcIdxSubset.front()]) - m_ExtGssRange) : 0;
	double TgtMinGss = (fabs(pGssTgt[TgtIdxSubset.front()]) - m_ExtGssRange) > 0 ? (fabs(pGssTgt[TgtIdxSubset.front()]) - m_ExtGssRange) : 0;
	double SrcStableMaxGss = fabs(pGssSrc[*(SrcIdxSubset.end() - 5)]) + m_ExtGssRange;
	double TgtStableMaxGss = fabs(pGssTgt[*(TgtIdxSubset.end() - 5)]) + m_ExtGssRange;
	double SrcGssRange[2] = { SrcMinGss, SrcStableMaxGss };
	double TgtGssRange[2] = { TgtMinGss, TgtStableMaxGss };

	std::vector<unsigned>::const_iterator BItrSrc = std::lower_bound(SrcIdxSubset.begin(), SrcIdxSubset.end(), TgtGssRange[0], IdxCmp<double>(pGssSrc));
	std::vector<unsigned>::const_iterator EItrSrc = std::upper_bound(SrcIdxSubset.begin(), SrcIdxSubset.end(), TgtGssRange[1], IdxCmp<double>(pGssSrc));
	std::vector<unsigned>::const_iterator BItrTgt = std::lower_bound(TgtIdxSubset.begin(), TgtIdxSubset.end(), SrcGssRange[0], IdxCmp<double>(pGssTgt));
	std::vector<unsigned>::const_iterator EItrTgt = std::upper_bound(TgtIdxSubset.begin(), TgtIdxSubset.end(), SrcGssRange[1], IdxCmp<double>(pGssTgt));

	unsigned int RangeSrc = EItrSrc - BItrSrc;
	unsigned int RangeTgt = EItrTgt - BItrTgt;
	vioSrcSubset->swapIndexSet(std::vector<unsigned>(BItrSrc, EItrSrc), false);
	vioTgtSubset->swapIndexSet(std::vector<unsigned>(BItrTgt, EItrTgt), false);

#ifdef _DEBUG
	std::ofstream ResetSrcGssFile("TestData\\output\\Gss\\SrcCurv_reset.txt");
	std::ofstream ResetTgtGssFile("TestData\\output\\Gss\\TgtCurv_reset.txt");
	for (std::vector<unsigned int>::const_iterator CItr=vioSrcSubset->getIndexSet().begin(); CItr!=vioSrcSubset->getIndexSet().end(); ++CItr)
	{
		ResetSrcGssFile << pGssSrc[*CItr] << std::endl;
	}
	for (std::vector<unsigned int>::const_iterator CItr=vioTgtSubset->getIndexSet().begin(); CItr!=vioTgtSubset->getIndexSet().end(); ++CItr)
	{
		ResetTgtGssFile << pGssTgt[*CItr] << std::endl;
	}
	ResetSrcGssFile.close();
	ResetTgtGssFile.close();
#endif
}

//*********************************************************************************
//FUNCTION:
boost::shared_ptr<CSimilarityTriangle> hiveRegistration::CCorrespondenceEstimationASI::__createSimilarityTriangle(const CIndexSubset* vLoopIdxSet, const CIndexSubset* vMatchIdxSet) const
{
	std::vector<unsigned> LoopSampleKNN;
	std::vector<unsigned> MatchSampleKNN;
	__setSimilaritySamplePointSet(vLoopIdxSet, LoopSampleKNN);
	__setSimilaritySamplePointSet(vMatchIdxSet, MatchSampleKNN);
	return boost::shared_ptr<CSimilarityTriangle>(new CSimilarityTriangle(vLoopIdxSet->getPointCloud(), vMatchIdxSet->getPointCloud(), LoopSampleKNN, MatchSampleKNN));
}

//*********************************************************************************
//FUNCTION:
double hiveRegistration::CCorrespondenceEstimationASI::__comStepRatio(unsigned int vNumMatchSubset) const
{
	double MaxSectionRatio = 1.0 * m_MaxNumMatchPntEachStep / vNumMatchSubset;
	double SuggestSectionRatio = 1.0 / m_NumMatchSection;
	return (SuggestSectionRatio < MaxSectionRatio) ? SuggestSectionRatio : MaxSectionRatio;
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CCorrespondenceEstimationASI::__setCurLoopIdxRange(const double vCurCurvRange[2], const CIndexSubset* vLoopIdxSet, unsigned int voIdxRange[2]) const
{
	const double* pGss = vLoopIdxSet->getPointCloud().getGssCurvaturePointer();
	const std::vector<unsigned int>& InitIdxSet = vLoopIdxSet->getIndexSet();
	double CurvRange[2] = { (vCurCurvRange[0] - m_ExtGssRange*m_ExtendGssRangeFactor) > 0 ? (vCurCurvRange[0] - m_ExtGssRange*m_ExtendGssRangeFactor) : 0, vCurCurvRange[1] + m_ExtGssRange*m_ExtendGssRangeFactor };
	std::vector<unsigned>::const_iterator BItr = std::lower_bound(InitIdxSet.begin(), InitIdxSet.end(), CurvRange[0], IdxCmp<double>(pGss));
	std::vector<unsigned>::const_iterator EItr = std::upper_bound(InitIdxSet.begin(), InitIdxSet.end(), CurvRange[1], IdxCmp<double>(pGss));
	voIdxRange[0] = BItr - InitIdxSet.begin();
	voIdxRange[1] = EItr - InitIdxSet.begin() - 1;
}

//*********************************************************************************
//FUNCTION:
boost::shared_ptr<Eigen::Matrix<double, Eigen::Dynamic, 1> > hiveRegistration::CCorrespondenceEstimationASI::__getOrCreateCurCentImg(unsigned int vImgIdx, const CIndexSubset* vLoopIdxSet)
{
	boost::shared_ptr<Eigen::Matrix<double, Eigen::Dynamic, 1> > CurLoopImg;
	static CSpinImagesGenerator s_LoopSpinImagesGenerator(vLoopIdxSet->getPointCloud());
	BOOST_AUTO(FindItr, m_MapLoopImageData.find(vImgIdx));
	if (FindItr == m_MapLoopImageData.end())
	{
		CurLoopImg.reset(new Eigen::Matrix<double, Eigen::Dynamic, 1>());
		CurLoopImg->resize(m_ImageWidth*m_ImageHeight, 1);
		s_LoopSpinImagesGenerator.genImamge(vLoopIdxSet->getIndexSet()[vImgIdx], *CurLoopImg);
		m_MapLoopImageData.insert(std::make_pair(vImgIdx, CurLoopImg));
	}
	else
	{
		CurLoopImg = FindItr->second;
	}

	return CurLoopImg;
}

//*********************************************************************************
//FUNCTION:
bool hiveRegistration::CCorrespondenceEstimationASI::__comCurRT(const std::vector<std::pair<unsigned, std::vector<unsigned> > >& vIdxGroupSet, const CIndexSubset* vLoopIdxSet, CLCPTriangle& vLCPTriangle, Eigen::Matrix3d& voR, Eigen::Vector3d& voT)
{
	static double UnitSquareDist = (dynamic_cast<CUniqueData*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(CUniqueData::getSigSrcUniqueData())))->getUniqSquareDist();
	CTransformationEstimationLCP* pTmpRT = dynamic_cast<CTransformationEstimationLCP*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(CTransformationEstimationLCP::getClassSig()));
	bool IsMatchOK = vLCPTriangle.comRTWithLastElement(vIdxGroupSet, UnitSquareDist, voR, voT);
	if ((&(vLoopIdxSet->getPointCloud())) != m_pSourcePointCloud)
	{
		pTmpRT->fetchTra() = -voR.inverse() * voT;
		pTmpRT->fetchRot() = voR.inverse();
	}
	else
	{
		pTmpRT->fetchTra() = voT;
		pTmpRT->fetchRot() = voR;
	}

	return IsMatchOK;
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CCorrespondenceEstimationASI::__comCurMatchSpinImgs(const CIndexSubset* vMatchIdxSet, std::vector<unsigned int>::const_iterator vMatchItrRange[2], Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& voImgs) const
{
	static CSpinImagesGenerator s_MatchSpinImageGenerator(vMatchIdxSet->getPointCloud());
	s_MatchSpinImageGenerator.genSpinImages(std::vector<unsigned int>(vMatchItrRange[0], vMatchItrRange[1]), voImgs);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CCorrespondenceEstimationASI::__comCurMatchItrRange(const CIndexSubset* vMatchIdxSet, double vSectionRatio, std::vector<unsigned int>::const_iterator vioMatchItrRange[2]) const
{
	const std::vector<unsigned int>& InitIdxSet = vMatchIdxSet->getIndexSet();
	unsigned int NumCurSection = InitIdxSet.size() * vSectionRatio;
	if ((InitIdxSet.begin() + NumCurSection) < vioMatchItrRange[1])
	{
		vioMatchItrRange[0] = vioMatchItrRange[1] - NumCurSection;
	}
	else
	{
		vioMatchItrRange[0] = InitIdxSet.begin();
	}
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CCorrespondenceEstimationASI::__comCurGssCurvRange(const CIndexSubset* vMatchIdxSet, std::vector<unsigned int>::const_iterator vMatchItrRange[2], double voCurCurvRange[2]) const
{
	unsigned int CurIdxRange[2] = { *vMatchItrRange[0], *(vMatchItrRange[1] - 1) };
	voCurCurvRange[0] = fabs(vMatchIdxSet->getPointCloud().getGssCurvaturePointer()[ CurIdxRange[0] ]);
	voCurCurvRange[1] = fabs(vMatchIdxSet->getPointCloud().getGssCurvaturePointer()[ CurIdxRange[1] ]);
}