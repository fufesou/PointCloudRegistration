#include "CorrespondenceEstimationLCSNSClosest.h"
#include "ProductFactory.h"
#include "ProductFactoryData.h"
#include "ControlParameters.h"
#include "ICPMacros.h"
#include "RegUtilityFunctions.h"
#include "CommonLCSNSSubset.hpp"
#include "AuxiliaryConstraintLCSNSClosest.h"
#include "CorrespondenceEstimationLCSNSClosest.h"
#include "NormalShootingLCSNSClosest.h"

using namespace hiveRegistration;

hiveCommon::CProductFactory<CCorrespondenceEstimationLCSNSClosest> TheCreator(CCorrespondenceEstimationLCSNSClosest::getClassSig());

CCorrespondenceEstimationLCSNSClosest::CCorrespondenceEstimationLCSNSClosest()
: m_pNormalShootingLCSNS(boost::shared_ptr<CNormalShootingLCSNSClosest>(new CNormalShootingLCSNSClosest))
{
	__parseConfig();

	if (m_DoAxuiliaryEstimation) m_pAuxiliaryConstraintLCSNS = boost::shared_ptr<CAuxiliaryConstraintLCSNSClosest>(new CAuxiliaryConstraintLCSNSClosest);
}

CCorrespondenceEstimationLCSNSClosest::~CCorrespondenceEstimationLCSNSClosest()
{
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CCorrespondenceEstimationLCSNSClosest::_setPointCloudV( const hivePointCloud::CPointCloud* vSourcePointCloud, const hivePointCloud::CPointCloud* vTargetPointCloud )
{
	_ASSERT(vSourcePointCloud && vTargetPointCloud);

	m_pSourcePointCloud = vSourcePointCloud;
	m_pTargetPointCloud = vTargetPointCloud;

	m_pNormalShootingLCSNS->setTgtPointCloud(vTargetPointCloud);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CCorrespondenceEstimationLCSNSClosest::__parseConfig(void)
{
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyDoAxuiliaryEstimation(), m_DoAxuiliaryEstimation);
	_ASSERT(DumpRes);
}

//*********************************************************************************
//FUNCTION:
void CCorrespondenceEstimationLCSNSClosest::_determineCorrespondencesV(
	const CorrespondencePointSet& vCorrespondencePointSet, const Eigen::Matrix3d& vR, const ColVector3& vT,
	CorrespondencePairSet& voCorrespondencePairs)
{
	CorrespondencePairSet InitCorrespondencePairSet;
	m_pNormalShootingLCSNS->determineCorrespondencePairSet(vCorrespondencePointSet, vR, vT, InitCorrespondencePairSet);

	std::cout << InitCorrespondencePairSet.first->getPointSet().size() << std::endl;

	if (m_DoAxuiliaryEstimation)
	{
		m_pAuxiliaryConstraintLCSNS->doAuxiliaryPairConstraint(InitCorrespondencePairSet, voCorrespondencePairs);
	}
	else
	{
		voCorrespondencePairs = InitCorrespondencePairSet;
	}
}