#include "SamplerRandom.h"
#include <vector>
#include "HiveCommonMicro.h"
#include "ICPMacros.h"
#include "ProductFactory.h"
#include "ProductFactoryData.h"
#include "ControlParameters.h"
#include "PointCloudSubset.h"


hiveCommon::CProductFactory<hiveRegistration::CSamplerRandom> TheCreator(hiveRegistration::CSamplerRandom::getClassSig());

//*********************************************************************************
//FUNCTION:
hiveRegistration::CSamplerRandom::CSamplerRandom(void)
: m_SampleNum(-1)
, m_SampleRatio(-1.0)
{
	_letFactoryReleaseProduct();
	__parseConfig();
}

//*********************************************************************************
//FUNCTION:
hiveRegistration::CorrespondencePointSet hiveRegistration::CSamplerRandom::_doSampleV(const hivePointCloud::CPointCloud& vPointCloud)
{
	std::vector<unsigned int> IdxSubset;
	double SampleRatio = -1.0;
	int NumPnt = vPointCloud.getNumPoints();

	if (m_SampleRatio > 0.0 && m_SampleRatio < 1.0)
	{
		SampleRatio = m_SampleRatio;
	}
	else if (m_SampleNum > 0 && m_SampleNum < NumPnt)
	{
		SampleRatio = 1.0 * m_SampleNum / NumPnt;
	}
	else
	{
		return makeSharedPtr((CBasePointCloudSubset*)NULL);
	}

//#pragma omp parallel for
	for (int i=0; i<NumPnt; ++i)		// warning: not safe here
	{
		_HIVE_SIMPLE_IF(ICP_RAND(0.0, 1.0) < SampleRatio, IdxSubset.push_back(i));
	}

	return makeSharedPtr(new CIndexSubset(vPointCloud, IdxSubset));
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CSamplerRandom::__parseConfig(void)
{
	bool DumpNum = PTR_CONTROL_PARAMS->dumpValue(getKeySampleNum(), m_SampleNum);
	bool DumpRatio = PTR_CONTROL_PARAMS->dumpValue(getKeySampleRatio(), m_SampleRatio);
	_ASSERT(DumpNum || DumpRatio);
}
