#pragma once
#include "BaseProduct.h"
#include "ICPType.h"

namespace hiveRegistration
{
	class CBaseTransformationEstimation  : public hiveCommon::CBaseProduct
	{
	public:
		void computeRotationAndTranslation(const CorrespondencePairSet& vCorrespondencePairs, double* vioR, double* vioT)
		{
			Eigen::Matrix3d RMat3;
			::memcpy(RMat3.data(), vioR, 3*3*sizeof(double));
			ColVector3 TVec3;
			::memcpy(TVec3.data(), vioR, 1*3*sizeof(double));

			computeRotationAndTranslation(vCorrespondencePairs, RMat3, TVec3);

			::memcpy(vioR, RMat3.data(), 3*3*sizeof(double));
			::memcpy(vioT, TVec3.data(), 1*3*sizeof(double));
		}

		void computeRotationAndTranslation(const CorrespondencePairSet& vCorrespondencePairs, Eigen::Matrix3d& vioR, ColVector3& vioT)
		{
			_computeRotationAndTranslationV(vCorrespondencePairs, vioR, vioT);
		}

		void accumulateTransformation(Eigen::Matrix3d& vioR, ColVector3& vioT, const Eigen::Matrix3d& vCurR, const ColVector3& vCurT)
		{
			vioR = vCurR * vioR;
			vioT = vCurR * vioT + vCurT;
		}

	protected:
		virtual ~CBaseTransformationEstimation() {}

		virtual void _computeRotationAndTranslationV(const CorrespondencePairSet& vCorrespondencePairs, Eigen::Matrix3d& vioR, ColVector3& vioT) = 0;
	};
}