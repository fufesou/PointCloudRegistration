#pragma once
#include <boost/algorithm/string.hpp>
#include "ICPType.h"
#include "BaseCorrespondenceEstimation.h"

namespace hivePointCloud
{
	class CPointCloud;
}

namespace hiveRegistration
{
	class CNormalShootingLCSNSClosest;
	class CAuxiliaryConstraintLCSNSClosest;

	class CCorrespondenceEstimationLCSNSClosest : public CBaseCorrespondenceEstimation
	{
		friend class boost::serialization::access;

	public:
		CCorrespondenceEstimationLCSNSClosest();

		static std::string getClassSig() { return boost::algorithm::to_upper_copy(std::string("CorrespondenceEstimationLCSNSClosest")); }
		static std::string getKeyDoAxuiliaryEstimation() { return boost::algorithm::to_upper_copy(std::string("CorrespondenceEstimationLCSNSClosest.DAE")); }

	protected:
		~CCorrespondenceEstimationLCSNSClosest();

		virtual void _setPointCloudV(const hivePointCloud::CPointCloud* vSourcePointCloud, const hivePointCloud::CPointCloud* vTargetPointCloud) override;
		virtual void _determineCorrespondencesV(const CorrespondencePointSet& vCorrespondencePointSet, const Eigen::Matrix3d& vR, const ColVector3& vT, 
			CorrespondencePairSet& voCorrespondencePairs) override;

	private:
		int m_DoAxuiliaryEstimation;

		boost::shared_ptr<CNormalShootingLCSNSClosest>      m_pNormalShootingLCSNS;
		boost::shared_ptr<CAuxiliaryConstraintLCSNSClosest> m_pAuxiliaryConstraintLCSNS;

		void __parseConfig(void);
	};
}