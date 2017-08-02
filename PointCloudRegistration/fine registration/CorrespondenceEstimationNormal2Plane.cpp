#include "CorrespondenceEstimationNormal2Plane.h"
#include "ProductFactory.h"
#include "ProductFactoryData.h"
#include "ControlParameters.h"
#include "ICPMacros.h"
#include "RegUtilityFunctions.h"
#include "PointCloudSubset.h"
#include "EstimationAuxiliaryPairConstraint.h"
#include "PairEstimationMethodNormal2Plane.h"

#ifdef _DEBUG
#include "TestUnitity.h"
#endif

using namespace hiveRegistration;

hiveCommon::CProductFactory<CCorrespondenceEstimationNormal2Plane> TheCreator(CCorrespondenceEstimationNormal2Plane::getClassSig());

CCorrespondenceEstimationNormal2Plane::CCorrespondenceEstimationNormal2Plane()
: m_pPairEstimationNormal2Plane(boost::shared_ptr<CPairEstimationNormal2Plane>(new CPairEstimationNormal2Plane))
, m_pEstimationAuxiliaryPairConstraint(boost::shared_ptr<CEstimationAuxiliaryPairConstraint>(new CEstimationAuxiliaryPairConstraint))
{
}

CCorrespondenceEstimationNormal2Plane::~CCorrespondenceEstimationNormal2Plane()
{
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CCorrespondenceEstimationNormal2Plane::_setPointCloudV( const hivePointCloud::CPointCloud* vSourcePointCloud, const hivePointCloud::CPointCloud* vTargetPointCloud )
{
	_ASSERT(vSourcePointCloud && vTargetPointCloud);

	m_pSourcePointCloud = vSourcePointCloud;
	m_pTargetPointCloud = vTargetPointCloud;
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CCorrespondenceEstimationNormal2Plane::__parseConfig( void )
{
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyDoAuxiliaryConstraint(), m_DoAuxiliaryConstraint);
	_ASSERT(DumpRes);
}

//*********************************************************************************
//FUNCTION:
void CCorrespondenceEstimationNormal2Plane::_determineCorrespondencesV(
	const CorrespondencePointSet& vTargetCorrespondencePointSet, const Eigen::Matrix3d& vR, const ColVector3& vT,
	CorrespondencePairSet& voCorrespondencePairs)
{
	CorrespondencePairSet InitCorrespondencePairSet;
	m_pPairEstimationNormal2Plane->determineCorrespondencePairSet(vTargetCorrespondencePointSet, m_pSourcePointCloud, vR, vT, InitCorrespondencePairSet);

	std::cout << InitCorrespondencePairSet.first->getPointSet().size() << std::endl;

	if (m_DoAuxiliaryConstraint)
	{
		m_pEstimationAuxiliaryPairConstraint->doAuxiliaryPairConstraint(InitCorrespondencePairSet, voCorrespondencePairs);
	}
	else
	{
		voCorrespondencePairs = InitCorrespondencePairSet;
	}

// #ifdef _DEBUG
// 	const VecColVector3& SrcPnts = voCorrespondencePairs.first->getPointSet();
// 	const VecColVector3& TgtPnts = voCorrespondencePairs.second->getPointSet();
// 	static int s_Int = 0;
// 	++s_Int;
// 	static char FileName[255];
// 	sprintf(FileName, "TestData\\output\\CorPointPairs\\FineReg_Mid\\All\\%d_src.ply", s_Int);
// 	export2Ply(FileName, SrcPnts);
// 	sprintf(FileName, "TestData\\output\\CorPointPairs\\FineReg_Mid\\All\\%d_tgt.ply", s_Int);
// 	export2Ply(FileName, TgtPnts);
// #endif
}
