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
	class CEstimationAuxiliaryPairConstraint;
	class CPairEstimationNormal2Plane;

	class CCorrespondenceEstimationNormal2Plane : public CBaseCorrespondenceEstimation
	{
		friend class boost::serialization::access;

	public:
		CCorrespondenceEstimationNormal2Plane();

		static std::string getClassSig() { return boost::algorithm::to_upper_copy(std::string("CorrespondenceEstimationNormal2Plane")); }
		static std::string getKeyDoAuxiliaryConstraint() { return boost::algorithm::to_upper_copy(std::string("CorrespondenceEstimationNormal2Plane.DAC")); }

	protected:
		~CCorrespondenceEstimationNormal2Plane();

		virtual void _setPointCloudV(const hivePointCloud::CPointCloud* vSourcePointCloud, const hivePointCloud::CPointCloud* vTargetPointCloud) override;
		virtual void _determineCorrespondencesV(const CorrespondencePointSet& vTargetCorrespondencePointSet, const Eigen::Matrix3d& vR, const ColVector3& vT, 
			CorrespondencePairSet& voCorrespondencePairs) override;

	private:
		int m_DoAuxiliaryConstraint;
		const hivePointCloud::CPointCloud*						m_pSourcePointCloud;
		const hivePointCloud::CPointCloud*						m_pTargetPointCloud;
		boost::shared_ptr<CPairEstimationNormal2Plane>			m_pPairEstimationNormal2Plane;
		boost::shared_ptr<CEstimationAuxiliaryPairConstraint>	m_pEstimationAuxiliaryPairConstraint;

		void __parseConfig(void);
	};
}