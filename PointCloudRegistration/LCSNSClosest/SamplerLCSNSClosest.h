#pragma once
#include <boost/serialization/access.hpp>
#include <boost/tuple/tuple.hpp>
#include "BaseSampler.h"
#include "LCSNSType.h"

namespace hiveRegistration
{
	class CSamplerLCSNSClosest : public CBaseSampler
	{
		friend boost::serialization::access;

	public:
		CSamplerLCSNSClosest();

		static std::string getClassSig()       { return boost::algorithm::to_upper_copy(std::string("SamplerLCSNSClosest")); }
		static std::string getKeyCorPointNum() { return boost::algorithm::to_upper_copy(std::string("SamplerLCSNSClosest.NumK")); }

	protected:
		virtual ~CSamplerLCSNSClosest();

		virtual CorrespondencePointSet _doSampleV(const hivePointCloud::CPointCloud& vPointCloud) override;

	private:
		CorrespondencePointSet m_pAllCenterPointSet;

		void __initAllCenterPointSet(const hivePointCloud::CPointCloud& vSourcePointCloud);
	};
}