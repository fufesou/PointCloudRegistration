#pragma once
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/serialization/access.hpp>
#include "BaseCorrespondenceRejection.h"
#include "ControlPointsData.h"

namespace hiveRegistration
{
	class CCorrespondenceRejectionCurvature : public CBaseCorrespondenceRejection
	{
		friend class boost::serialization::access;

	public:
		CCorrespondenceRejectionCurvature();

		static std::string getClassSig()     { return boost::algorithm::to_upper_copy(std::string("CorrespondenceRejectionCurvature")); }
		static std::string getKeyCurvatureConstraintThreshold() { return boost::algorithm::to_upper_copy(std::string("CorrespondenceRejectionCurvature.CCT")); }
		static std::string getKeyDoCurvatureConstraint() { return boost::algorithm::to_upper_copy(std::string("CorrespondenceRejectionCurvature.DCC")); }

	protected:
		~CCorrespondenceRejectionCurvature();

		virtual void _rejectInvalidPairsV(const CorrespondencePairSet& vAllCorrespondencePairs, CorrespondencePairSet& voValidCorrespondencePairs) override;

	private:
		double m_CurvatureConstraintThreshold;
		int m_DoDoCurvatureConstraint;

		void __computePrincipalCurvatureSet(const std::vector<std::pair<double, double> >& vUVSet, const std::vector<std::pair<int, int> >& vSurfaceIndexPairSet, 
			const hiveRegistration::MatrixControlPoints &vControlPoints, std::vector<std::pair<double, double> >& voPrincipalCurvatureSet) const;

		void __parseCfg(void);
	};
}
