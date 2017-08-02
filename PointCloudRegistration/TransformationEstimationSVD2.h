#pragma once
#include <boost/algorithm/string.hpp>
#include <Eigen/Core>
#include "BaseTransformationEstimation.h"
#include "PointCloud.h"

namespace hivePointCloud
{
	class CPointCloud;
}

namespace hiveRegistration
{
	class CTransformationEstimationSVD2 : public CBaseTransformationEstimation
	{
		friend class boost::serialization::access;

	public:
		CTransformationEstimationSVD2();

		static std::string getClassSig() { return boost::algorithm::to_upper_copy(std::string("TransformationEstimationSVD2")); }

		void computeRotationAndTranslation(const VecColVector3& vPntSetA, const VecColVector3& vPntSetB, Eigen::Matrix3d& voR, ColVector3& voT);

	protected:
		~CTransformationEstimationSVD2() {}

		virtual void _computeRotationAndTranslationV(const CorrespondencePairSet& vCorrespondencePairs, Eigen::Matrix3d& vioR, ColVector3& vioT) override;

	private:
		void __deMean(const VecColVector3& vSrcPointSet, const VecColVector3& vTgtPointSet, const std::pair<ColVector3, ColVector3>& vCentroidPair, CorrespondencePairSet& voDemeanedCorPairSet);
		void __buildPointPairMatrix(const CorrespondencePairSet& vCorPairSet, Eigen::Matrix<double, 3, Eigen::Dynamic>& voPointMatA, Eigen::Matrix<double, 3, Eigen::Dynamic>& voPointMatB);
		void __computeRelativeRotationAndTranslation(
			const Eigen::Matrix<double, 3, Eigen::Dynamic>& vSrcDemeanMat, const Eigen::Matrix<double, 3, Eigen::Dynamic>& vTgtDemeanMat, 
			const std::pair<ColVector3, ColVector3>& vCentroidPair,
			Eigen::Matrix3d& voCurR, ColVector3& voCurT);
	};
}