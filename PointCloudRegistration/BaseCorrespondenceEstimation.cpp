#include "BaseCorrespondenceEstimation.h"
#include "PointCloudSubset.h"

using namespace hiveRegistration;

CBaseCorrespondenceEstimation::CBaseCorrespondenceEstimation()
: m_pSourcePointCloud(NULL)
, m_pTargetPointCloud(NULL)
{

}

//*********************************************************************************
//FUNCTION:
void CBaseCorrespondenceEstimation::determineCorrespondences(const CorrespondencePointSet& vSourceCorrespondencePointSet, const Eigen::Matrix3d& vR, const ColVector3& vT, CorrespondencePairSet& vioCorrespondencePairs)
{
	_ASSERT(vSourceCorrespondencePointSet->getPointSet().size() > 0);

	_determineCorrespondencesV(vSourceCorrespondencePointSet, vR, vT, vioCorrespondencePairs);
}

//*********************************************************************************
//FUNCTION:
void CBaseCorrespondenceEstimation::setPointCloud(const hivePointCloud::CPointCloud* vSourcePointCloud, const hivePointCloud::CPointCloud* vTargetPointCloud)
{
	_ASSERT(vSourcePointCloud && vTargetPointCloud);

	if ((vSourcePointCloud != m_pSourcePointCloud) && (vTargetPointCloud != m_pTargetPointCloud))
	{
		_setPointCloudV(vSourcePointCloud, vTargetPointCloud);
	}
}
