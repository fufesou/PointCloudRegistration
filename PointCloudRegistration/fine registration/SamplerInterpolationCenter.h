#pragma once
#include <boost/serialization/access.hpp>
#include <boost/algorithm/string.hpp>
#include "ControlPointsData.h"
#include "BaseSampler.h"

namespace hiveRegistration
{
	class CSamplerInterpolationCenter : public CBaseSampler
	{
		friend boost::serialization::access;

	public:
		CSamplerInterpolationCenter();

		static std::string getClassSig()       { return boost::to_upper_copy(std::string("SamplerInterpolationCenter")); }
		static std::string getKeyCorPointNum() { return boost::to_upper_copy(std::string("SamplerInterpolationCenter.NumK")); }

protected:
		virtual ~CSamplerInterpolationCenter();

		virtual CorrespondencePointSet _doSampleV(const hivePointCloud::CPointCloud& vPointCloud) override;

private:
		CorrespondencePointSet m_pAllCenterPointSet;

		void __initAllCenterPointSet(const hivePointCloud::CPointCloud& vSourcePointCloud);
		void __findInterpolationSurfaceCenterSet(const MatrixControlPoints& vControlPoints, VecColVector3& voPointSet, std::vector<std::pair<int, int> >& voIndexPairSet, VecColVector3& voNormalSet);
		CorrespondencePointSet __setUpCorrespondencePointSet();
	};
}