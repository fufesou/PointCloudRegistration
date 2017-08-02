#include "TransformationEstimationLCP.h"
#include "ProductFactory.h"


hiveCommon::CProductFactory<hiveRegistration::CTransformationEstimationLCP> TheCreator(hiveRegistration::CTransformationEstimationLCP::getClassSig());

hiveRegistration::CTransformationEstimationLCP::CTransformationEstimationLCP()
{
	_letFactoryReleaseProduct();
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CTransformationEstimationLCP::_computeRotationAndTranslationV(const CorrespondencePairSet& vCorrespondencePairs, Eigen::Matrix3d& vioR, ColVector3& vioT)
{
	vioR = m_Rot;
	vioT = m_Tra;
}