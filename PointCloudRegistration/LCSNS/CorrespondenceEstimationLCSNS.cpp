#include "CorrespondenceEstimationLCSNS.h"
#include "ProductFactory.h"
#include "ProductFactoryData.h"
#include "ControlParameters.h"
#include "ICPMacros.h"
#include "RegUtilityFunctions.h"
#include "CommonLCSNSSubset.hpp"
#include "AuxiliaryConstraintLCSNS.h"
#include "CorrespondenceEstimationLCSNS.h"
#include "NormalShootingLCSNS.h"


using namespace hiveRegistration;

hiveCommon::CProductFactory<CCorrespondenceEstimationLCSNS> TheCreator(CCorrespondenceEstimationLCSNS::getClassSig());

CCorrespondenceEstimationLCSNS::CCorrespondenceEstimationLCSNS()
: m_pNormalShootingLCSNS(boost::shared_ptr<CNormalShootingLCSNS>(new CNormalShootingLCSNS))
{
	__parseConfig();

	if (m_DoAxuiliaryEstimation) m_pAuxiliaryConstraintLCSNS = boost::shared_ptr<CAuxiliaryConstraintLCSNS>(new CAuxiliaryConstraintLCSNS);
}

CCorrespondenceEstimationLCSNS::~CCorrespondenceEstimationLCSNS()
{
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CCorrespondenceEstimationLCSNS::_setPointCloudV( const hivePointCloud::CPointCloud* vSourcePointCloud, const hivePointCloud::CPointCloud* vTargetPointCloud )
{
	_ASSERT(vSourcePointCloud && vTargetPointCloud);

	m_pSourcePointCloud = vSourcePointCloud;
	m_pTargetPointCloud = vTargetPointCloud;


}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CCorrespondenceEstimationLCSNS::__parseConfig(void)
{
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyDoAxuiliaryEstimation(), m_DoAxuiliaryEstimation);
	_ASSERT(DumpRes);
}

//*********************************************************************************
//FUNCTION:
void CCorrespondenceEstimationLCSNS::_determineCorrespondencesV(
	const CorrespondencePointSet& vCorrespondencePointSet, const Eigen::Matrix3d& vR, const ColVector3& vT,
	CorrespondencePairSet& voCorrespondencePairs)
{
	CorrespondencePairSet InitCorrespondencePairSet;
	m_pNormalShootingLCSNS->determineCorrespondencePairSet(vCorrespondencePointSet, m_pSourcePointCloud, vR, vT, InitCorrespondencePairSet);

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