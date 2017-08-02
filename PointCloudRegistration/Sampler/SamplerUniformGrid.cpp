#include "SampleUniformGrid.h"
#include <vector>
#include "HiveCommonMicro.h"
#include "ICPMacros.h"
#include "ProductFactory.h"
#include "ProductFactoryData.h"
#include "ControlParameters.h"
#include "PointCloudSubset.h"


hiveCommon::CProductFactory<CSampleUniformGrid> TheCreator(CSampleUniformGrid::getClassSig());

//*********************************************************************************
//FUNCTION:
hiveRegistration::CSampleUniformGrid::CSampleUniformGrid(void)
: m_SampleNum(-1)
, m_SampleRatio(-1.0)
{
	_letFactoryReleaseProduct();
	__parseConfig();
}

//*********************************************************************************
//FUNCTION:
hiveRegistration::CorrespondencePointSet hiveRegistration::CSampleUniformGrid::_doSampleV(const hivePointCloud::CPointCloud& vPointCloud)
{

}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CSampleUniformGrid::__parseConfig(void)
{
	bool DumpNum = PTR_CONTROL_PARAMS->dumpValue(getKeySampleNum(), m_SampleNum);
	bool DumpRatio = PTR_CONTROL_PARAMS->dumpValue(getKeySampleRatio(), m_SampleRatio);
	_ASSERT(DumpNum || DumpRatio);
}
