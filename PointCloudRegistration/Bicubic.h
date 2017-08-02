#pragma once
#include "ControlPointsData.h"

namespace Bicubic
{
	class CBicubic
	{
	public:
		CBicubic();
		virtual ~CBicubic();

		void computeNormal(double vU, double vV, const Eigen::Matrix4d vControlPoints[], hiveRegistration::ColVector3& voNormal) const;
		void computePrincipleCurvatures(double vU, double vV, const Eigen::Matrix4d vControlPoints[], std::pair<double, double>& voPrincipleCurvatures) const;
		void compute3DPointPosByUV(double vU, double vV, const Eigen::Matrix4d vControlPoints[], hiveRegistration::ColVector3& voPos) const;
		void intersectRay(const Eigen::Matrix4d vCtrlPoints[], const hiveRegistration::ColVector3& vRayOrigin, const hiveRegistration::ColVector3& vRayDir, hiveRegistration::ColVector2& voEndMatrix, bool& voIntersected) const;
	
	private:

	};
}