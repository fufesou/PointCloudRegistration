#include "Registration.h"
#include <string>
#include "ProductFactoryData.h"
#include "HiveCommonMicro.h"
#include "EventLoggerInterface.h"
#include "ControlParameters.h"
#include "ICPMacros.h"
#include "ICPConstGlobleValue.h"
#include "IterativeFit.h"
#include "BaseConvergenceCriteria.h"
#include "DefaultConvergenceCriterial.h"
#include "SamplerExtremeCurvature.h"
#include "CorrespondenceEstimationExtremeCurvature.h"
#include "CorrespondenceRejectionLCP.h"
#include "SamplerInterpolationCenter.h"
#include "CorrespondenceEstimationNormal2Plane.h"
#include "CorrespondenceRejectionCurvature.h"
#include "TransformationEstimationSVD2.h"

using namespace hiveRegistration;

CRegistartion::CRegistartion()
: m_pCoarseIterativeFit(new CIterativeFit)
, m_pFineIterativeFit(new CIterativeFit)
{
	__init();
}

//*********************************************************************************
//FUNCTION:
void CRegistartion::__init()
{
	__parseConfigFile();
	__fillICPClassesSig();
}

//*********************************************************************************
//FUNCTION:
void CRegistartion::__parseConfigFile()
{
	PTR_CONTROL_PARAMS->parseConfig(g_ConfigFile);
}

//*********************************************************************************
//FUNCTION:
void CRegistartion::__setCoarseFitClasses()
{
	std::string CoarseSamplerSig;
	std::string CoarseCorrespondenceEstimationSig;
	std::string CoarseCorrespondenceRejectionSig;
	std::string CoarseTransformationEstimationSig;
	PTR_CONTROL_PARAMS->dumpValue(g_CoarseSampler, CoarseSamplerSig);
	PTR_CONTROL_PARAMS->dumpValue(g_CoarseCorrespondenceEstimation, CoarseCorrespondenceEstimationSig);
	PTR_CONTROL_PARAMS->dumpValue(g_CoarseCorrespondenceRejection, CoarseCorrespondenceRejectionSig);
	PTR_CONTROL_PARAMS->dumpValue(g_CoarseTransformationEstimation, CoarseTransformationEstimationSig);

	m_pCoarseIterativeFit->setFitClasses(CoarseSamplerSig, CoarseCorrespondenceEstimationSig, CoarseCorrespondenceRejectionSig, CoarseTransformationEstimationSig);
}

//*********************************************************************************
//FUNCTION:
void CRegistartion::__setFineFitClasses()
{
	std::string FineSamplerSig;
	std::string FineCorrespondenceEstimationSig;
	std::string FineCorrespondenceRejectionSig;
	std::string FineTransformationEstimationSig;
	PTR_CONTROL_PARAMS->dumpValue(g_FineSampler, FineSamplerSig);
	PTR_CONTROL_PARAMS->dumpValue(g_FineCorrespondenceEstimation, FineCorrespondenceEstimationSig);
	PTR_CONTROL_PARAMS->dumpValue(g_FineCorrespondenceRejection, FineCorrespondenceRejectionSig);
	PTR_CONTROL_PARAMS->dumpValue(g_FineTransformationEstimation, FineTransformationEstimationSig);

	m_pFineIterativeFit->setFitClasses(FineSamplerSig, FineCorrespondenceEstimationSig, FineCorrespondenceRejectionSig, FineTransformationEstimationSig);
}

//*********************************************************************************
//FUNCTION:
void CRegistartion::__fillICPClassesSig()
{
	std::string Tmp;

	if (!PTR_CONTROL_PARAMS->dumpValue(g_CoarseSampler, Tmp))
	{
		PTR_CONTROL_PARAMS->set(g_CoarseSampler, CSamplerExtremeCurvature::getClassSig());
	}
	if (!PTR_CONTROL_PARAMS->dumpValue(g_CoarseCorrespondenceEstimation, Tmp))
	{
		PTR_CONTROL_PARAMS->set(g_CoarseCorrespondenceEstimation, CCorrespondenceEstimationExtremeCurvature::getClassSig());
	}
	if (!PTR_CONTROL_PARAMS->dumpValue(g_CoarseCorrespondenceRejection, Tmp))
	{
		PTR_CONTROL_PARAMS->set(g_CoarseCorrespondenceRejection, CCorrespondenceRejectionLCP::getClassSig());
	}
	if (!PTR_CONTROL_PARAMS->dumpValue(g_CoarseTransformationEstimation, Tmp))
	{
		PTR_CONTROL_PARAMS->set(g_CoarseTransformationEstimation, CTransformationEstimationSVD2::getClassSig());
	}

	if (!PTR_CONTROL_PARAMS->dumpValue(g_FineSampler, Tmp))
	{
		PTR_CONTROL_PARAMS->set(g_FineSampler, CSamplerInterpolationCenter::getClassSig());
	}
	if (!PTR_CONTROL_PARAMS->dumpValue(g_FineCorrespondenceEstimation, Tmp))
	{
		PTR_CONTROL_PARAMS->set(g_FineCorrespondenceEstimation, CCorrespondenceEstimationNormal2Plane::getClassSig());
	}
	if (!PTR_CONTROL_PARAMS->dumpValue(g_FineCorrespondenceRejection, Tmp))
	{
		PTR_CONTROL_PARAMS->set(g_FineCorrespondenceRejection, CCorrespondenceRejectionCurvature::getClassSig());
	}
	if (!PTR_CONTROL_PARAMS->dumpValue(g_FineTransformationEstimation, Tmp))
	{
		PTR_CONTROL_PARAMS->set(g_FineTransformationEstimation, CTransformationEstimationSVD2::getClassSig());
	}

}

//*********************************************************************************
//FUNCTION:
CRegistartion::RotTraCoarseFineFitMsg CRegistartion::fit(
	const hivePointCloud::CPointCloud& vSourcePointCloud, 
	const hivePointCloud::CPointCloud& vTargetPointCloud, 
	const Eigen::Matrix3d& vInitR, 
	ColVector3& vInitT)
{
	RotTraFitMsg RotTraCoarseFitMsg = coarseFit(vSourcePointCloud, vTargetPointCloud, vInitR, vInitT);
	RotTraFitMsg RotTraFineFitMsg = fineFit(vSourcePointCloud, vTargetPointCloud, RotTraCoarseFitMsg.get<0>(), RotTraCoarseFitMsg.get<1>());

	return boost::make_tuple(RotTraFineFitMsg.get<0>(), RotTraFineFitMsg.get<1>(), RotTraCoarseFitMsg.get<2>(), RotTraFineFitMsg.get<2>());
}

//*********************************************************************************
//FUNCTION:
CRegistartion::RotTraFitMsg CRegistartion::coarseFit(
	const hivePointCloud::CPointCloud& vSourcePointCloud, 
	const hivePointCloud::CPointCloud& vTargetPointCloud, 
	const Eigen::Matrix3d& vInitR, 
	ColVector3& vInitT)
{
	__parseConfigFile();
	__setCoarseFitClasses();

	return m_pCoarseIterativeFit->fit(vSourcePointCloud, vTargetPointCloud, vInitR, vInitT);
}

//*********************************************************************************
//FUNCTION:
CRegistartion::RotTraFitMsg CRegistartion::fineFit(
	const hivePointCloud::CPointCloud& vSourcePointCloud,
	const hivePointCloud::CPointCloud& vTargetPointCloud, 
	const Eigen::Matrix3d& vInitR,
	ColVector3& vInitT)
{
	__parseConfigFile();
	__setFineFitClasses();

	return m_pFineIterativeFit->fit(vSourcePointCloud, vTargetPointCloud, vInitR, vInitT);
}

//*********************************************************************************
//FUNCTION:
void CRegistartion::setSampler(const std::string& vPointSamplerSig)
{
	if (vPointSamplerSig.find(g_CoarseRegistrationPrefix) == 0)
	{
		PTR_CONTROL_PARAMS->set(g_CoarseSampler, vPointSamplerSig.substr(g_CoarseRegistrationPrefix.size()));
	}
	else if (vPointSamplerSig.find(g_FineRegistrationPrefix) == 0)
	{
		PTR_CONTROL_PARAMS->set(g_FineSampler, vPointSamplerSig.substr(g_FineRegistrationPrefix.size()));
	}
	else
	{
		hiveEventLogger::hiveOutputWarning(__EXCEPTION_SITE__, "failed to set sampler class!");
	}
}

//*********************************************************************************
//FUNCTION:
void CRegistartion::setCorrespondenceEstimation(const std::string& vCorrespondenceEsitmationSig)
{
	if (vCorrespondenceEsitmationSig.find(g_CoarseRegistrationPrefix) == 0)
	{
		PTR_CONTROL_PARAMS->set(g_CoarseCorrespondenceEstimation, vCorrespondenceEsitmationSig.substr(g_CoarseRegistrationPrefix.size()));
	}
	else if (vCorrespondenceEsitmationSig.find(g_FineRegistrationPrefix) == 0)
	{
		PTR_CONTROL_PARAMS->set(g_FineCorrespondenceEstimation, vCorrespondenceEsitmationSig.substr(g_FineRegistrationPrefix.size()));
	}
	else
	{
		hiveEventLogger::hiveOutputWarning(__EXCEPTION_SITE__, "failed to set correspondence estimation class!");
	}
}

//*********************************************************************************
//FUNCTION:
void CRegistartion::setCorrespondenceRejection(const std::string& vCorrespondenceRejectionSig)
{
	if (vCorrespondenceRejectionSig.find(g_CoarseRegistrationPrefix) == 0)
	{
		PTR_CONTROL_PARAMS->set(g_CoarseCorrespondenceRejection, vCorrespondenceRejectionSig.substr(g_CoarseRegistrationPrefix.size()));
	}
	else if (vCorrespondenceRejectionSig.find(g_FineRegistrationPrefix) == 0)
	{
		PTR_CONTROL_PARAMS->set(g_FineCorrespondenceRejection, vCorrespondenceRejectionSig.substr(g_FineRegistrationPrefix.size()));
	}
	else
	{
		hiveEventLogger::hiveOutputWarning(__EXCEPTION_SITE__, "failed to set correspondence rejection class!");
	}
}

//*********************************************************************************
//FUNCTION:
void CRegistartion::setTransformationEstimation(const std::string& vTransformationEsitmationSig)
{
	if (vTransformationEsitmationSig.find(g_CoarseRegistrationPrefix) == 0)
	{
		PTR_CONTROL_PARAMS->set(g_CoarseTransformationEstimation, vTransformationEsitmationSig.substr(g_CoarseRegistrationPrefix.size()));
	}
	else if (vTransformationEsitmationSig.find(g_FineRegistrationPrefix) == 0)
	{
		PTR_CONTROL_PARAMS->set(g_FineTransformationEstimation, vTransformationEsitmationSig.substr(g_FineRegistrationPrefix.size()));
	}
	else
	{
		hiveEventLogger::hiveOutputWarning(__EXCEPTION_SITE__, "failed to set transformation estimation class!");
	}
}