#pragma once
#include <boost/serialization/access.hpp>
#include <boost/tuple/tuple.hpp>
#include "BaseSampler.h"
#include "LCSNSType.h"

namespace hiveRegistration
{
	class CSamplerLCSNS : public CBaseSampler
	{
		friend boost::serialization::access;

	public:
		CSamplerLCSNS();

		static std::string getClassSig()       { return boost::algorithm::to_upper_copy(std::string("SamplerLCSNS")); }
		static std::string getKeyCorPointNum() { return boost::algorithm::to_upper_copy(std::string("SamplerLCSNS.NumK")); }

	protected:
		virtual ~CSamplerLCSNS();

		virtual CorrespondencePointSet _doSampleV(const hivePointCloud::CPointCloud& vPointCloud) override;

	private:
		CorrespondencePointSet m_pAllCenterPointSet;

		void __initAllCenterPointSet(const hivePointCloud::CPointCloud& vSourcePointCloud);
	};
}