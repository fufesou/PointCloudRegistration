#pragma once
#include <boost/serialization/access.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/shared_ptr.hpp>
#include "BaseCorrespondenceEstimation.h"

namespace hiveCommon
{
	class CKNNSearch;
}

namespace hivePointCloud
{
	class CPointCloud;
}

namespace hiveRegistration
{
	class CCorrespondenceEstimationClosestPoints : public CBaseCorrespondenceEstimation
	{
		friend class boost::serialization::access;

	public:
		CCorrespondenceEstimationClosestPoints(void);

		static std::string getClassSig() { return boost::to_upper_copy(std::string("CorrespondenceEstimationClosestPoints")); }
		static std::string getKeyNumNeighbors() { return boost::to_upper_copy(std::string("CorrespondenceEstimationClosestPoints.NN")); }
		static std::string getKeyNormalDeviationThreshold() { return boost::to_upper_copy(std::string("CorrespondenceEstimationClosestPoints.NDT")); }
		static std::string getKeyNormalVectorDistThreshold() { return boost::to_upper_copy(std::string("CorrespondenceEstimationClosestPoints.NVDT")); }
		static std::string getKeySquareDistFactorThreshold() { return boost::to_upper_copy(std::string("CorrespondenceEstimationClosestPoints.SDFT")); }
		static std::string getKeyNeighbourDistFactorThreshold() { return boost::to_upper_copy(std::string("CorrespondenceEstimationClosestPoints.NDFT")); }

	protected:
		virtual ~CCorrespondenceEstimationClosestPoints(void) {}

		virtual void _setPointCloudV(const hivePointCloud::CPointCloud* vSourcePointCloud, const hivePointCloud::CPointCloud* vTargetPointCloud) override;
		virtual void _determineCorrespondencesV(const CorrespondencePointSet& vSourceCorrespondencePointSet, const Eigen::Matrix3d& vR, const ColVector3& vT, CorrespondencePairSet& vioCorrespondencePairs) override;

	private:
		unsigned int m_NumNeighbors;
		double m_NormalDeviationThreshold;
		double m_NormalVectorDistThreshold;
		double m_SquareDistFactorThreshold;
		double m_NeighbourDistFactorThreshold;
		double m_UniqueSquareDist;
		boost::shared_ptr<hiveCommon::CKNNSearch> m_pPointCloudKDTree;

		void __updateKDTree();
		void __insertFitPairs(ColVector3& vSrcCorPoint, ColVector3& vSrcCorNormal, const std::vector<unsigned int>& vNeighbors,
			VecColVector3& vioPairedSrcPointSet, VecColVector3& vioPairedSrcNormalSet,
			VecColVector3& vioPairedTgtPointSet, VecColVector3& vioPairedTgtNormalSet);

		void __parseConfig(void);
	};
}