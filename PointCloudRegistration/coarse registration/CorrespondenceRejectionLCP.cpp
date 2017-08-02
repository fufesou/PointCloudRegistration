#include "CorrespondenceRejectionLCP.h"
#include "ProductFactory.h"


using namespace hiveRegistration;

hiveCommon::CProductFactory<CCorrespondenceRejectionLCP> TheCreator(CCorrespondenceRejectionLCP::getClassSig());

hiveRegistration::CCorrespondenceRejectionLCP::CCorrespondenceRejectionLCP()
{
	_letFactoryReleaseProduct();
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CCorrespondenceRejectionLCP::_rejectInvalidPairsV(const CorrespondencePairSet& vAllCorrespondencePairs, CorrespondencePairSet& voValidCorrespondencePairs)
{

}
