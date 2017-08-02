#pragma once
#include <utility>
#include <vector>
#include <boost\shared_ptr.hpp>
#include "BaseProduct.h"
#include "ICPType.h"

namespace hivePointCloud
{
	class CPointCloud;
}

namespace hiveRegistration
{
	class CBaseCorrespondenceEstimation : public hiveCommon::CBaseProduct
	{
	public:
		CBaseCorrespondenceEstimation();

		void setPointCloud(const hivePointCloud::CPointCloud* vSourcePointCloud, const hivePointCloud::CPointCloud* vTargetPointCloud);
		void determineCorrespondences(const CorrespondencePointSet& vTgtCorPointSet, const Eigen::Matrix3d& vR, const ColVector3& vT, CorrespondencePairSet& vioCorrespondencePairs);

	protected:
		const hivePointCloud::CPointCloud* m_pSourcePointCloud;
		const hivePointCloud::CPointCloud* m_pTargetPointCloud;

		virtual ~CBaseCorrespondenceEstimation(void) {}

		virtual void _setPointCloudV(const hivePointCloud::CPointCloud* vSourcePointCloud, const hivePointCloud::CPointCloud* vTargetPointCloud) = 0;
		virtual void _determineCorrespondencesV(const CorrespondencePointSet& vTgtCorPointSet, const Eigen::Matrix3d& vR, const ColVector3& vT, CorrespondencePairSet& vioCorrespondencePairs) = 0;		
	};
}