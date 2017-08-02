#pragma once
#include "BaseProduct.h"
#include "ICPType.h"

namespace hivePointCloud
{
	class CPointCloud;
}

namespace hiveRegistration
{
	class CBaseCorrespondenceRejection : public hiveCommon::CBaseProduct
	{
	public:
		CBaseCorrespondenceRejection() : m_pSourcePointCloud(NULL), m_pTargetPointCloud(NULL) {}

		void setPointCloud(const hivePointCloud::CPointCloud* vSourcePointCloud, const hivePointCloud::CPointCloud* vTargetPointCloud)
		{
			if (vSourcePointCloud != m_pSourcePointCloud) _setSrcPointCloudV(vSourcePointCloud);
			if (vTargetPointCloud != m_pTargetPointCloud) _setTgtPointCloudV(vTargetPointCloud);
		}

		void rejectInvalidPairs(const CorrespondencePairSet& vAllCorrespondencePairs, CorrespondencePairSet& voValidCorrespondencePairs)
		{
			_rejectInvalidPairsV(vAllCorrespondencePairs, voValidCorrespondencePairs);
		}

	protected:
		const hivePointCloud::CPointCloud* m_pSourcePointCloud;
		const hivePointCloud::CPointCloud* m_pTargetPointCloud;

		virtual ~CBaseCorrespondenceRejection() {}

		virtual void _rejectInvalidPairsV(const CorrespondencePairSet& vAllCorrespondencePairs, CorrespondencePairSet& voValidCorrespondencePairs) = 0;

		virtual void _setSrcPointCloudV(const hivePointCloud::CPointCloud* vSrcPointCloud) { m_pSourcePointCloud = vSrcPointCloud; }
		virtual void _setTgtPointCloudV(const hivePointCloud::CPointCloud* vTgtPointCloud) { m_pTargetPointCloud = vTgtPointCloud; }

	private:
	};
}