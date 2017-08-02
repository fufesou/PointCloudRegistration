#include "CorrespondenceRejectionClosestPoints.h"
#include "ProductFactory.h"
#include "PointCloud.h"
#include "ICPType.h"
#include "PointCloudSubset.h"

using namespace hiveRegistration;

hiveCommon::CProductFactory<CCorrespondenceRejectionClosestPoints> TheCreator(CCorrespondenceRejectionClosestPoints::getClassSig());

CCorrespondenceRejectionClosestPoints::CCorrespondenceRejectionClosestPoints(void)
{
	_letFactoryReleaseProduct();
}

//*********************************************************************************
//FUNCTION:
void CCorrespondenceRejectionClosestPoints::_rejectInvalidPairsV(const CorrespondencePairSet& vAllCorrespondencePairs, CorrespondencePairSet& voValidCorrespondencePairs)
{
	std::cout << vAllCorrespondencePairs.first->getPointSet().size() << std::endl;
	voValidCorrespondencePairs = vAllCorrespondencePairs;
}