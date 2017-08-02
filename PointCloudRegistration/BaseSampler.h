#pragma once
#include <vector>
#include "BaseProduct.h"
#include "ICPType.h"

namespace hivePointCloud
{
	class CPointCloud;
}

namespace hiveRegistration
{
	class CBaseSampler : public hiveCommon::CBaseProduct
	{
	public:
		CorrespondencePointSet samplePointCloud(const hivePointCloud::CPointCloud& vPointCloud)
		{
			return _doSampleV(vPointCloud);
		}

	protected:
		virtual ~CBaseSampler(void) {}

		virtual CorrespondencePointSet _doSampleV(const hivePointCloud::CPointCloud& vPointCloud) = 0;
	};
}