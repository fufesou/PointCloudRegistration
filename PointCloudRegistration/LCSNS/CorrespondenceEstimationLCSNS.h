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
	class CAuxiliaryConstraintLCSNS;
	class CNormalShootingLCSNS;
	class CNormalShootingLCSNSClosest;

	class CCorrespondenceEstimationLCSNS : public CBaseCorrespondenceEstimation
	{
		friend class boost::serialization::access;

	public:
		CCorrespondenceEstimationLCSNS();

		static std::string getClassSig()                 { return boost::algorithm::to_upper_copy(std::string("CorrespondenceEstimationLCSNS")); }
		static std::string getKeyDoAxuiliaryEstimation() { return boost::algorithm::to_upper_copy(std::string("CorrespondenceEstimationLCSNS.DAE")); }

	protected:
		~CCorrespondenceEstimationLCSNS();

		virtual void _setPointCloudV(const hivePointCloud::CPointCloud* vSourcePointCloud, const hivePointCloud::CPointCloud* vTargetPointCloud) override;
		virtual void _determineCorrespondencesV(const CorrespondencePointSet& vCorrespondencePointSet, const Eigen::Matrix3d& vR, const ColVector3& vT, 
			CorrespondencePairSet& voCorrespondencePairs) override;

	private:
		int m_DoAxuiliaryEstimation;

		boost::shared_ptr<CNormalShootingLCSNS>      m_pNormalShootingLCSNS;
		boost::shared_ptr<CAuxiliaryConstraintLCSNS> m_pAuxiliaryConstraintLCSNS;

		void __parseConfig(void);
	};
}