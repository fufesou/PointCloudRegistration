#include "IterativeFit.h"
#include <boost/lexical_cast.hpp>
#include <boost/tuple/tuple.hpp>
#include "ProductFactoryData.h"
#include "HiveCommonMicro.h"
#include "EventLoggerInterface.h"
#include "ControlParameters.h"
#include "ICPMacros.h"
#include "PointCloud.h"
#include "PointCloudSubset.h"
#include "BaseSampler.h"
#include "BaseCorrespondenceEstimation.h"
#include "BaseCorrespondenceRejection.h"
#include "BaseTransformationEstimation.h"
#include "DefaultConvergenceCriterial.h"
#include "TestUnitity.h"
#include "UniqueData.h"

using namespace hiveRegistration;

//*********************************************************************************
//FUNCTION:
void CIterativeFit::setFitClasses(const std::string& vPointSamplerSig, const std::string& vCorrespondenceEsitmationSig, const std::string& vCorrespondenceRejectionSig, const std::string& vTransformationEsitmationSig)
{
	setSampler(vPointSamplerSig);
	setCorrespondenceEstimation(vCorrespondenceEsitmationSig);
	setCorrespondenceRejection(vCorrespondenceRejectionSig);
	setTransformationEstimation(vTransformationEsitmationSig);
	setConvergenceCriterial(CDefaultConvergenceCriteria::getClassSig());
}

//*********************************************************************************
//FUNCTION:
void CIterativeFit::setSampler(const std::string& vPointSamplerSig)
{
	m_pPointSampler = dynamic_cast<CBaseSampler*>( hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(vPointSamplerSig) );
	_ASSERT(m_pPointSampler);
}

//*********************************************************************************
//FUNCTION:
void CIterativeFit::setCorrespondenceEstimation(const std::string& vCorrespondenceEsitmationSig)
{
	m_pCorrespondenceEstimation = dynamic_cast<CBaseCorrespondenceEstimation*>( hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(vCorrespondenceEsitmationSig) );
	_ASSERT(m_pCorrespondenceEstimation);
}

//*********************************************************************************
//FUNCTION:
void CIterativeFit::setCorrespondenceRejection(const std::string& vCorrespondenceRejectionSig)
{
	m_pCorrespondenceRejection = dynamic_cast<CBaseCorrespondenceRejection*>( hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(vCorrespondenceRejectionSig) );
	_ASSERT(m_pCorrespondenceRejection);
}

//*********************************************************************************
//FUNCTION:
void CIterativeFit::setTransformationEstimation(const std::string& vTransformationEsitmationSig)
{
	m_pTransformationEsitmation = dynamic_cast<CBaseTransformationEstimation*>( hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(vTransformationEsitmationSig) );
	_ASSERT(m_pTransformationEsitmation);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CIterativeFit::setConvergenceCriterial(const std::string& vConvergenceCriterialSig)
{
	m_pConvergenceCriterial = dynamic_cast<CDefaultConvergenceCriteria*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(vConvergenceCriterialSig));
	_ASSERT(m_pConvergenceCriterial);
}

//*********************************************************************************
//FUNCTION:
boost::tuple<Eigen::Matrix3d, ColVector3, std::string> CIterativeFit::fit(
	const hivePointCloud::CPointCloud& vSourcePointCloud, 
	const hivePointCloud::CPointCloud& vTargetPointCloud, 
	const Eigen::Matrix3d& vInitR, 
	const ColVector3& vInitT)
{
	__setPointCloud(vSourcePointCloud, vTargetPointCloud);
	__setMethodPointCloud();

	Eigen::Matrix3d Rot = vInitR;
	ColVector3 Tra = vInitT;

	_HIVE_EARLY_RETURN(!_isCorrespondencePointSetValidV(), "control point cannot be less than 5.", boost::make_tuple(Rot, Tra, "Point Cloud Not Valid!"));

	__fitLoop(Rot, Tra);

	std::string ConvergenceMsg = __getConvergenceMsg();

	return boost::make_tuple(Rot, Tra, ConvergenceMsg);
}

//*********************************************************************************
//FUNCTION:
void CIterativeFit::_updateV()
{
	int SampleTarget;
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeySamplePointCloud(), SampleTarget);
	_ASSERT(DumpRes);

	m_SampleSubset = m_pPointSampler->samplePointCloud(* (SampleTarget ? m_pTargetPointCloud : m_pSourcePointCloud) );
}

//*********************************************************************************
//FUNCTION:
bool CIterativeFit::_isCorrespondencePointSetValidV()
{
	if ( (!m_SampleSubset.get()) || (0 == m_SampleSubset->getPointSet().size()))
	{
		_updateV();
	}
	return (m_SampleSubset->getPointSet().size() >= 5);	// FIXME: 要把求采样是否有效的方法写到采样结果里去，而不应该在这里判断
}

//*********************************************************************************
//FUNCTION:
void CIterativeFit::__fitLoop(Eigen::Matrix3d& vioR, ColVector3& vioT)
{
	unsigned int IterationCounter = 0;

	while (FIT_END != _fitStepV(++IterationCounter, vioR, vioT))
	{
#ifdef _DEBUG
		std::cout << "iteration counter: " << IterationCounter << "\n" << "rotation: \n" << vioR << "\n translation:\n" << vioT << "\n" << std::endl;
#endif
	}
}

//*********************************************************************************
//FUNCTION:
CIterativeFit::EFitFlag CIterativeFit::_fitStepV(unsigned int vIterationCounter, Eigen::Matrix3d& vioR, ColVector3& vioT)
{
	CorrespondencePairSet AllCorrespondencePairSet;
	CorrespondencePairSet ValidCorrespondencePairSet;

	m_pCorrespondenceEstimation->determineCorrespondences(m_SampleSubset, vioR, vioT, AllCorrespondencePairSet);
	//_ASSERT(AllCorrespondencePairSet.first->getPointSet().size() > 5);

// 	if (AllCorrespondencePairSet.first->getPointSet().size() < 5)
// 	{
// 		std::cout << "Registration failed!" << std::endl;
// 		return FIT_END;
// 	}

	m_pCorrespondenceRejection->rejectInvalidPairs(AllCorrespondencePairSet, ValidCorrespondencePairSet);

//	_ASSERT(ValidCorrespondencePairSet.first->getPointSet().size() > 5);
// 	if (ValidCorrespondencePairSet.first->getPointSet().size() < 5)
// 	{
// 		std::cout << "Registration failed!" << std::endl;
// 		return FIT_END;
// 	}

	m_pTransformationEsitmation->computeRotationAndTranslation(ValidCorrespondencePairSet, vioR, vioT);

	m_pConvergenceCriterial->setCorrespondencePairSet(ValidCorrespondencePairSet);
	bool IsConverged = m_pConvergenceCriterial->isConverged(vioR, vioT, vIterationCounter);

#ifdef _DEBUG
	static int MidCloudCounter = 0;
	hivePointCloud::CPointCloud MidPointCloud = *m_pSourcePointCloud;
	MidPointCloud.transform(vioR.data(), vioT.data());
	export2Ply((std::string(".//TestData//output//MidPointCloud//MidSrc") + boost::lexical_cast<std::string>(MidCloudCounter) + std::string(".ply")).c_str(), MidPointCloud);
	++MidCloudCounter;
#endif

#ifndef _DEBUG
	static int s_ItrCount = 0;
	std::cout << ++s_ItrCount << std::endl;
#endif
	
	return IsConverged ? FIT_END : FIT_NOT_END;
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CIterativeFit::__setPointCloud(const hivePointCloud::CPointCloud& vSourcePointCloud, const hivePointCloud::CPointCloud& vTargetPointCloud)
{
	_ASSERT(vSourcePointCloud.getPosPointer() && vTargetPointCloud.getPosPointer());

	CUniqueData* pSrcData = dynamic_cast<CUniqueData*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(CUniqueData::getSigSrcUniqueData()));
	CUniqueData* pTgtData = dynamic_cast<CUniqueData*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(CUniqueData::getSigTgtUniqueData()));
	_ASSERT(pSrcData && pTgtData);
	pSrcData->setPointCloud(vSourcePointCloud);
	pTgtData->setPointCloud(vTargetPointCloud);

	m_pSourcePointCloud = &vSourcePointCloud;
	m_pTargetPointCloud = &vTargetPointCloud;
}

//*********************************************************************************
//FUNCTION:
std::string hiveRegistration::CIterativeFit::__getConvergenceMsg()
{
	return m_pConvergenceCriterial->getConvergenceMsg();
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CIterativeFit::__setMethodPointCloud()
{
	m_pCorrespondenceEstimation->setPointCloud(m_pSourcePointCloud, m_pTargetPointCloud);
	m_pCorrespondenceRejection->setPointCloud(m_pSourcePointCloud, m_pTargetPointCloud);
}
