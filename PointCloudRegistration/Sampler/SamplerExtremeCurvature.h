#pragma once
#include <boost/algorithm/string.hpp>
#include <boost/serialization/access.hpp>
#include "BaseSampler.h"
#include "ICPType.h"


namespace hiveRegistration
{
	class CSamplerExtremeCurvature : public CBaseSampler
	{
		friend boost::serialization::access;

	public:
		CSamplerExtremeCurvature(void);

		static std::string getClassSig()                  { return boost::to_upper_copy(std::string("SamplerExtremeCurvature")); }
		static std::string getKeyExtremePointNumNeib()    { return boost::to_upper_copy(std::string("SamplerExtremeCurvature.EPNN")); }
		static std::string getKeyExtremePointDistFactor() { return boost::to_upper_copy(std::string("SamplerExtremeCurvature.EPDF")); }

		void setSampleDistFactor(double vFactor) { m_SampleDistFactor = vFactor; }
		void setSampleNumNeib(unsigned vNumNeib) { m_SampleNumNeib = vNumNeib; }
		unsigned getSampleNumNeib(void) const  { return m_SampleNumNeib; }
		double getSampleSquareDist(void) const { return m_UnitSquareDist*m_SampleDistFactor; }

	protected:
		virtual ~CSamplerExtremeCurvature(void) {}

		virtual CorrespondencePointSet _doSampleV(const hivePointCloud::CPointCloud& vPointCloud) override;

	private:
		double m_UnitSquareDist;
		double m_SampleDistFactor;
		unsigned m_SampleNumNeib;
		unsigned m_FirstSampleNumNeib;
		unsigned m_SecondSampleNumNeib;

		void __parseConfig(void);
		void __init(void);
		void __simpleSample(const hivePointCloud::CPointCloud& vPointCloud, std::vector<unsigned>& voSubset) const;
		void __firstSample(const hivePointCloud::CPointCloud& vPointCloud, std::vector<unsigned>& voSubset) const;
		void __secondSample(const hivePointCloud::CPointCloud& vPointCloud, std::vector<unsigned>& vioSubset) const;
		bool __isExtremeCurvaturePoint(const hivePointCloud::CPointCloud& vPointCloud, unsigned int vIdx, const std::vector<unsigned>& vNeibs) const;
		bool __isExtremeCurvaturePointWithinDist(const hivePointCloud::CPointCloud& vPointCloud, unsigned int vIdx, const std::vector<unsigned>& vNeibs) const;
	};
}
