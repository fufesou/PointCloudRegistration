#include "DefaultConvergenceCriterial.h"
#include "ProductFactory.h"
#include "ProductFactoryData.h"
#include "ControlParameters.h"
#include "ICPMacros.h"
#include "PointCloudSubset.h"
#include "UniqueData.h"

using namespace hiveRegistration;

hiveCommon::CProductFactory<CDefaultConvergenceCriteria> TheCreator(CDefaultConvergenceCriteria::getClassSig());

CDefaultConvergenceCriteria::CDefaultConvergenceCriteria()
{
	_letFactoryReleaseProduct();
	__init();
}

//*********************************************************************************
//FUNCTION:
void CDefaultConvergenceCriteria::__init()
{
	m_IterationsSimilarTransforms = 0;
	m_PreMSE = std::numeric_limits<double>::max();
	m_CurMSE = std::numeric_limits<double>::max();

	CControlParameters* pConfig = PTR_CONTROL_PARAMS;
	pConfig->setIfNotExist(getKeyFailureAfterMaxIterations(), false);
	pConfig->setIfNotExist(getKeyRotationThreshold(), 0.0001);
	pConfig->setIfNotExist(getKeyTranslationThreshold(), 3e-8);
	pConfig->setIfNotExist(getKeyRelativeMSE(), 0.01);
	pConfig->setIfNotExist(getKeyAbsoluteMSE(), 0.01);
	pConfig->setIfNotExist(getKeyMaxIterationSimilarTransforms(), unsigned int(0));
	pConfig->setIfNotExist(getKeyConvergenceState(), static_cast<char>(CONVERGENCE_CRITERIA_NOT_CONVERGED));

	setMapConvergenceType(static_cast<char>(CONVERGENCE_CRITERIA_NOT_CONVERGED), "NotConverged");
	setMapConvergenceType(static_cast<char>(CONVERGENCE_CRITERIA_ITERATIONS), "Criteria_Iterations");
	setMapConvergenceType(static_cast<char>(CONVERGENCE_CRITERIA_TRANSFORM), "Criteria_Transform");
	setMapConvergenceType(static_cast<char>(CONVERGENCE_CRITERIA_ABS_MSE), "Criteria_AbsMSE");
	setMapConvergenceType(static_cast<char>(CONVERGENCE_CRITERIA_REL_MSE), "Criteria_RelMSE");
	setMapConvergenceType(static_cast<char>(CONVERGENCE_CRITERIA_NO_CORRESPONDENCES), "Criteria_NoCorrespondeces");
}

//*********************************************************************************
//FUNCTION:
double CDefaultConvergenceCriteria::__calculateMSE() const
{
	_ASSERT(m_pCorrespondencePairs);

	double MSEValue = 0;
	const VecColVector3& SourcePointSet = (m_pCorrespondencePairs->first)->getPointSet();
	const VecColVector3& TargetPointSet = (m_pCorrespondencePairs->second)->getPointSet();
	const unsigned int SizeSourcePointSet = SourcePointSet.size();
	_ASSERT(SizeSourcePointSet == TargetPointSet.size());
	_ASSERT(SizeSourcePointSet != 0);

	for (unsigned int i=0; i!=SizeSourcePointSet; ++i)
	{
		MSEValue += (SourcePointSet[i] - TargetPointSet[i]).squaredNorm();
	}

	MSEValue /= static_cast<double>(SizeSourcePointSet);

	return MSEValue;
}

//*********************************************************************************
//FUNCTION:
bool CDefaultConvergenceCriteria::_isConvergedV(const Eigen::Matrix3d& vCurR, const ColVector3& vCurT, unsigned int vIterationCounter)
{
	CControlParameters* pCfg = dynamic_cast<CControlParameters*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(CControlParameters::getClassSig()));
	_ASSERT(pCfg);

	pCfg->set(getKeyConvergenceState(), static_cast<char>(CONVERGENCE_CRITERIA_NOT_CONVERGED));

	bool IsConverged =  (__isIterationEndedV(vIterationCounter) || __isTransformationEnded(vCurR, vCurT) || __isMSEAbsoluteEnded() || __isMSERelativeEnded());
	m_PreMSE = m_CurMSE;
	m_PreRot = vCurR;
	m_PreTra = vCurT;

	return IsConverged;
}

//*********************************************************************************
//FUNCTION:
bool CDefaultConvergenceCriteria::__isIterationEndedV(unsigned int vIterationCounter)
{
	bool FailureAfterMaxIter;
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyFailureAfterMaxIterations(), FailureAfterMaxIter);
	_ASSERT(DumpRes);

	bool isIterationEnded = CBaseConvergenceCriteria::__isIterationEndedV(vIterationCounter);
	if (isIterationEnded)
	{
		PTR_CONTROL_PARAMS->set(getKeyConvergenceState(), static_cast<char>(CONVERGENCE_CRITERIA_ITERATIONS));
		return FailureAfterMaxIter ? false : true;
	}
	return false;
}

//*********************************************************************************
//FUNCTION:
bool CDefaultConvergenceCriteria::__isTransformationEnded(const Eigen::Matrix3d& vCurR, const ColVector3& vCurT)
{
	double RotDiff = (m_PreRot - vCurR).squaredNorm();
	double TranslationSqr = vCurT.squaredNorm();

	double RotationThreshold;
	double TranslationThreshold;
	unsigned int MaxIterationSimilarTransforms;
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyRotationThreshold(), RotationThreshold);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyTranslationThreshold(), TranslationThreshold);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyMaxIterationSimilarTransforms(), MaxIterationSimilarTransforms);

	if (RotDiff < RotationThreshold && TranslationSqr < TranslationThreshold)
	{
		if (m_IterationsSimilarTransforms < MaxIterationSimilarTransforms)
		{
			++m_IterationsSimilarTransforms;
			return false;
		}
		else
		{
			m_IterationsSimilarTransforms = 0;
			PTR_CONTROL_PARAMS->set(getKeyConvergenceState(), static_cast<char>(CONVERGENCE_CRITERIA_TRANSFORM));
			return true;
		}
	}

	return false;
}

//*********************************************************************************
//FUNCTION:
bool CDefaultConvergenceCriteria::__isMSEAbsoluteEnded()
{
	double MSEThresholdAbsolute;
	unsigned int MaxIterationSimilarTransforms;
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyAbsoluteMSE(), MSEThresholdAbsolute);
	_ASSERT(DumpRes);

	CUniqueData* pUniqueData = dynamic_cast<CUniqueData*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(CUniqueData::getSigTgtUniqueData()));
	_ASSERT(pUniqueData);
	double UniqueSquareDist = pUniqueData->getUniqSquareDist();
	MSEThresholdAbsolute *= UniqueSquareDist;

	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyMaxIterationSimilarTransforms(), MaxIterationSimilarTransforms);
	_ASSERT(DumpRes);

	m_CurMSE = __calculateMSE();

	if (fabs (m_CurMSE - m_PreMSE) < MSEThresholdAbsolute)
	{
		if (m_IterationsSimilarTransforms < MaxIterationSimilarTransforms)
		{
			++m_IterationsSimilarTransforms;
			return false;
		}
		else
		{
			m_IterationsSimilarTransforms = 0;
			PTR_CONTROL_PARAMS->set(getKeyConvergenceState(), static_cast<char>(CONVERGENCE_CRITERIA_ABS_MSE));
			return true;
		}
	}

	return false;
}

//*********************************************************************************
//FUNCTION:
bool CDefaultConvergenceCriteria::__isMSERelativeEnded()
{
	double MSEThresholdRelative;
	unsigned int MaxIterationSimilarTransforms;
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyRelativeMSE(), MSEThresholdRelative);
	_ASSERT(DumpRes);
	
	CUniqueData* pUniqueData = dynamic_cast<CUniqueData*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(CUniqueData::getSigTgtUniqueData()));
	_ASSERT(pUniqueData);
	double UniqueSquareDist = pUniqueData->getUniqSquareDist();
	MSEThresholdRelative *= UniqueSquareDist;
	
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyMaxIterationSimilarTransforms(), MaxIterationSimilarTransforms);
	_ASSERT(DumpRes);

	m_CurMSE = __calculateMSE();

	if (fabs (m_CurMSE - m_PreMSE) / m_PreMSE < MSEThresholdRelative)
	{
		if (m_IterationsSimilarTransforms < MaxIterationSimilarTransforms)
		{
			++m_IterationsSimilarTransforms;
			return false;
		}
		else
		{
			m_IterationsSimilarTransforms = 0;
			PTR_CONTROL_PARAMS->set(getKeyConvergenceState(), static_cast<char>(CONVERGENCE_CRITERIA_REL_MSE));
			return true;
		}
	}

	return false;
}

//*********************************************************************************
//FUNCTION:
bool CDefaultConvergenceCriteria::operator()(const Eigen::Matrix3d& vCurR, const ColVector3& vCurT, unsigned int vIterationCounter)
{
	return _isConvergedV(vCurR, vCurT, vIterationCounter);
}

//*********************************************************************************
//FUNCTION:
bool CDefaultConvergenceCriteria::isConverged(const Eigen::Matrix3d& vCurR, const ColVector3& vCurT, unsigned int vIterationCounter)
{
	return _isConvergedV(vCurR, vCurT, vIterationCounter);
}