#pragma once
#include <boost/algorithm/string.hpp>
#include <boost/serialization/access.hpp>
#include "BaseSampler.h"
#include "ICPType.h"


namespace hiveRegistration
{
	class CSamplerRandom : public CBaseSampler
	{
		friend boost::serialization::access;

	public:
		CSamplerRandom(void);

		static std::string getClassSig()       { return boost::to_upper_copy(std::string("SamplerRandom")); }
		static std::string getKeySampleNum()   { return boost::to_upper_copy(std::string("SamplerRandom.SN")); }
		static std::string getKeySampleRatio() { return boost::to_upper_copy(std::string("SamplerRandom.SR")); }

	protected:
		virtual ~CSamplerRandom(void) {}

		virtual CorrespondencePointSet _doSampleV(const hivePointCloud::CPointCloud& vPointCloud) override;

	private:
		int    m_SampleNum;
		double m_SampleRatio;

		void __parseConfig(void);
	};
}
