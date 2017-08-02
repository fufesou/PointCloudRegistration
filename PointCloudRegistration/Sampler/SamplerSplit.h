#pragma once
#include <climits>
#include <boost/algorithm/string.hpp>
#include <boost/serialization/access.hpp>
#include "BaseSampler.h"
#include "ICPType.h"


namespace hiveRegistration
{
	class CSamplerSplit : public CBaseSampler
	{
		friend boost::serialization::access;

	public:
		CSamplerSplit(void);

		static std::string getClassSig()              { return boost::to_upper_copy(std::string("SamplerSplit")); }
		static std::string getKeySplitNodeMinNumPnt() { return boost::to_upper_copy(std::string("SamplerSplit.SNMNP")); }

	protected:
		virtual ~CSamplerSplit(void) {}

		virtual CorrespondencePointSet _doSampleV(const hivePointCloud::CPointCloud& vPointCloud) override;

	private:
		unsigned m_NodeMinNumPnt;
		bool m_UseCompactRegion;
		std::vector<unsigned> m_SampleIndexSet;
		const hivePointCloud::CPointCloud* m_pPointCloud;

		struct SplitNode 
		{
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

			RegionDim3 NodeRegion;
			std::vector<unsigned> ContainedPnts;

			SplitNode()
			{
				NodeRegion(0, 0) = -std::numeric_limits<double>::max();
				NodeRegion(1, 0) = -std::numeric_limits<double>::max();
				NodeRegion(2, 0) = -std::numeric_limits<double>::max();
				NodeRegion(0, 1) = std::numeric_limits<double>::max();
				NodeRegion(1, 1) = std::numeric_limits<double>::max();
				NodeRegion(2, 1) = std::numeric_limits<double>::max();
			}
		};

		typedef std::vector<SplitNode, Eigen::aligned_allocator<SplitNode> > VecSplitNode;

		void __parseConfig(void);
		void __init(const hivePointCloud::CPointCloud& vPointCloud);
		void __constructRoot(VecSplitNode& voSplitNodes);
		void __split(const VecSplitNode& vNodes2Split, VecSplitNode& voSplitNodes);
		void __doSample(VecSplitNode& vioSplitNodes);
		void __init8SplitNode(const RegionDim3& vParentRegion, const ColVector3& vCent, SplitNode voSplitNode[8]) const;
		void __updateRegion(const std::vector<unsigned>& vPointSet, RegionDim3& vioRegion) const;
		void __computeCentroid(const std::vector<unsigned>& vPointSet, ColVector3& voCentroid) const;
		void __computeRegionAndCentroid(const std::vector<unsigned>& vPointSet, RegionDim3& vioRegion, ColVector3& voCentroid) const;
		unsigned __findPointNearestCentroid(const std::vector<unsigned>& vPointSet, const ColVector3& vCentroid) const;
	};
}
