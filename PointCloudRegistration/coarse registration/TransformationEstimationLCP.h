#pragma once
#include <boost/algorithm/string.hpp>
#include <Eigen/Core>
#include "BaseTransformationEstimation.h"


namespace hiveRegistration
{
	class  CTransformationEstimationLCP : public CBaseTransformationEstimation
	{
		friend class boost::serialization::access;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		CTransformationEstimationLCP();

		static std::string getClassSig() { return boost::algorithm::to_upper_copy(std::string("TransformationEstimationLCP")); }

		const Eigen::Matrix3d& getRot() const { return m_Rot; }
		const Eigen::Vector3d& getTra() const { return m_Tra; }
		Eigen::Matrix3d& fetchRot() { return m_Rot; }
		Eigen::Vector3d& fetchTra() { return m_Tra; }

	protected:
		~CTransformationEstimationLCP() {}

		void _computeRotationAndTranslationV(const CorrespondencePairSet& vCorrespondencePairs, Eigen::Matrix3d& vioR, ColVector3& vioT) override;

	private:
		Eigen::Matrix3d m_Rot;
		Eigen::Vector3d m_Tra;
	};
}