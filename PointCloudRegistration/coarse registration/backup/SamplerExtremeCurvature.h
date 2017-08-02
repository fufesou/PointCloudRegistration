#pragma once
#include <boost/algorithm/string.hpp>
#include <boost/serialization/access.hpp>
#include "BaseSampler.h"
#include "ICPType.h"

namespace hiveCommon
{
	class CKNNSearch;
}

namespace hiveRegistration
{
	// 目前示未考虑点云在一个或两个轴上坐标相等的情况（平面或直线），三维点云匹配一般不会出现这种点云
	class CSamplerExtremeCurvature : public CBaseSampler
	{
		friend boost::serialization::access;

	public:
		CSamplerExtremeCurvature(void);

		static std::string getClassSig()                  { return boost::to_upper_copy(std::string("SamplerExtremeCurvature")); }
		static std::string getKeyExtremePointDistFactor() { return boost::to_upper_copy(std::string("SamplerExtremeCurvature.EPDF")); }
		static std::string getKeyExtremePointNumNeib()    { return boost::to_upper_copy(std::string("SamplerExtremeCurvature.EPNN")); }

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
		boost::shared_ptr<hiveCommon::CKNNSearch> m_pPointCloudKDTree;

		void __parseConfig(void);
		void __initPointCloudKDTree(const hivePointCloud::CPointCloud& vPointCloud);
		bool __isCurvatureExtremePoint(unsigned int vIdx, const hivePointCloud::CPointCloud& vPointCloud);
		bool __isMinCurvaturePoint(unsigned int vIdx, const hivePointCloud::CPointCloud& vPointCloud);
	};
}
